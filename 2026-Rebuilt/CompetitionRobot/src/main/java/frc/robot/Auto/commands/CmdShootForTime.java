package frc.robot.Auto.commands;

import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_BACKOFF;
import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_PULSE_DELAY_SEC;
import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_PULSE_ON_SEC;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FeederConstants.FeederSpeed;
import frc.robot.Constants.FloorConstants.ConveyorSpeed;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Logging.RobotLogger;
import frc.robot.Subsystems.Shooter;
import frc.robot.ShooterSpeeds;

/**
 * Command to feed and shoot for a specified duration.
 * 
 * <p>This command runs the feeder for the specified duration while the shooter
 * continues spinning. If requireAtSpeed is true, it first waits for the shooter
 * to reach target speed before feeding.
 * 
 * <p>The shooter and feeder run in parallel - the shooter continues spinning
 * while the feeder runs for the duration.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Feeding duration expires</li>
 * </ul>
 * 
 * @param shooter The shooter subsystem
 * @param feeder The feeder subsystem
 * @param durationSec Duration to feed in seconds
 * @param requireAtSpeed If true, wait for shooter at speed before feeding
 * @param targetRpmSupplier Supplier of target RPM (if requireAtSpeed is true)
 * @param rpmTol RPM tolerance (if requireAtSpeed is true)
 */
public class CmdShootForTime extends Command {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Floor floor;
    private final double durationSec;
    private final double spinupDelaySec;
    private final boolean requireAtSpeed;
    private final boolean useFeederBackoff;
    private final Supplier<Double> targetRpmSupplier;
    private final Supplier<ShooterSpeeds> targetSpeedsSupplier;
    private final Supplier<Double> shooterRpmSupplierForRun;
    private final Supplier<ShooterSpeeds> shooterSpeedsSupplierForRun;
    private final double rpmTol;
    private final Timer timer = new Timer();
    private Command feederCommand;

    /**
     * Full constructor with optional feeder backoff and optional floor.
     *
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor Optional floor subsystem; if non-null, floor runs in parallel with feeder and is stopped in end()
     * @param durationSec Duration to feed in seconds
     * @param spinupDelaySec Delay (seconds) before feeder/floor start, to let shooter stabilize
     * @param shooterRpmSupplierForRun Optional; when non-null, runs shooter at this speed for full duration (avoids parallel subsystem conflict)
     * @param requireAtSpeed If true, wait for shooter at speed before feeding
     * @param useFeederBackoff If true, run feeder reverse for {@code SHOOT_FEEDER_BACKOFF} first to clear ball from wheels
     * @param targetRpmSupplier Supplier of target RPM (required if requireAtSpeed is true)
     * @param rpmTol RPM tolerance (required if requireAtSpeed is true)
     */
    public CmdShootForTime(
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            double durationSec,
            double spinupDelaySec,
            Supplier<Double> shooterRpmSupplierForRun,
            Supplier<ShooterSpeeds> shooterSpeedsSupplierForRun,
            boolean requireAtSpeed,
            boolean useFeederBackoff,
            Supplier<Double> targetRpmSupplier,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
            double rpmTol) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.feeder = Objects.requireNonNull(feeder, "feeder cannot be null");
        this.floor = floor;
        if (durationSec <= 0) {
            throw new IllegalArgumentException("durationSec must be greater than 0, got: " + durationSec);
        }
        this.durationSec = durationSec;
        this.spinupDelaySec = spinupDelaySec;
        this.shooterRpmSupplierForRun = shooterRpmSupplierForRun;
        this.shooterSpeedsSupplierForRun = shooterSpeedsSupplierForRun;
        this.requireAtSpeed = requireAtSpeed;
        this.useFeederBackoff = useFeederBackoff;
        this.targetRpmSupplier = targetRpmSupplier;
        this.targetSpeedsSupplier = targetSpeedsSupplier;
        if (requireAtSpeed && targetRpmSupplier == null && targetSpeedsSupplier == null) {
            throw new NullPointerException("targetRpmSupplier or targetSpeedsSupplier required when requireAtSpeed is true");
        }
        this.rpmTol = rpmTol;

        addRequirements(shooter, feeder);
        if (floor != null) {
            addRequirements(floor);
        }
    }

    /** Legacy constructor (single RPM for shooter run and wait). */
    public CmdShootForTime(
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            double durationSec,
            double spinupDelaySec,
            Supplier<Double> shooterRpmSupplierForRun,
            boolean requireAtSpeed,
            boolean useFeederBackoff,
            Supplier<Double> targetRpmSupplier,
            double rpmTol) {
        this(shooter, feeder, floor, durationSec, spinupDelaySec,
                shooterRpmSupplierForRun, null, requireAtSpeed, useFeederBackoff,
                targetRpmSupplier, null, rpmTol);
    }

    /**
     * Creates a new CmdShootForTime command (no floor, no feeder backoff).
     *
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param durationSec Duration to feed in seconds
     * @param requireAtSpeed If true, wait for shooter at speed before feeding
     * @param targetRpmSupplier Supplier of target RPM (required if requireAtSpeed is true)
     * @param rpmTol RPM tolerance (required if requireAtSpeed is true)
     */
    public CmdShootForTime(
            Shooter shooter,
            Feeder feeder,
            double durationSec,
            boolean requireAtSpeed,
            Supplier<Double> targetRpmSupplier,
            double rpmTol) {
        this(shooter, feeder, null, durationSec, AutoConstants.SHOOT_SPINUP_DELAY_BEFORE_FEED, null, requireAtSpeed, false, targetRpmSupplier, rpmTol);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        RobotLogger.log(String.format("[CmdShootForTime] t=%.2f Started: duration=%.1fs spinupDelay=%.1fs",
                Timer.getFPGATimestamp(), durationSec, spinupDelaySec));

        Command pulseCycle = Commands.sequence(
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value)
                        .withTimeout(SHOOT_FEEDER_PULSE_ON_SEC)
                        .finallyDo(interrupted -> feeder.resetSpeed()),
                Commands.waitSeconds(SHOOT_FEEDER_PULSE_DELAY_SEC));
        Command feederPulsed = Commands.deadline(
                Commands.waitSeconds(durationSec),
                pulseCycle.repeatedly())
                .withName("CmdShootForTime-feeder");

        if (floor != null) {
            Command floorPhase = floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value)
                    .withTimeout(durationSec)
                    .withName("CmdShootForTime-floor");
            feederPulsed = new ParallelCommandGroup(feederPulsed, floorPhase);
        }
        Command feedPhase = feederPulsed;

        if (requireAtSpeed) {
            Supplier<ShooterSpeeds> speedsForWait = targetSpeedsSupplier != null
                    ? targetSpeedsSupplier
                    : () -> ShooterSpeeds.uniform(targetRpmSupplier.get());
            Command waitCommand = new CmdWaitShooterAtSpeed(shooter, speedsForWait, rpmTol, AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT);
            feedPhase = Commands.sequence(waitCommand, feedPhase);
        }

        // Extra spin-up time before feeder starts so shooter stabilizes
        feedPhase = Commands.sequence(
                Commands.waitSeconds(spinupDelaySec),
                feedPhase);

        // When shooter speed supplier is set, run shooter in parallel with feed phase
        if (shooterSpeedsSupplierForRun != null) {
            Command shooterRun = shooter.moveToTripleSpeedCommand(shooterSpeedsSupplierForRun)
                    .withTimeout(spinupDelaySec + durationSec)
                    .withName("CmdShootForTime-shooter");
            feedPhase = new ParallelCommandGroup(shooterRun, feedPhase);
        } else if (shooterRpmSupplierForRun != null) {
            Command shooterRun = shooter.moveToArbitrarySpeedCommand(shooterRpmSupplierForRun)
                    .withTimeout(spinupDelaySec + durationSec)
                    .withName("CmdShootForTime-shooter");
            feedPhase = new ParallelCommandGroup(shooterRun, feedPhase);
        }

        if (useFeederBackoff) {
            Command backoff = feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value)
                    .withTimeout(SHOOT_FEEDER_BACKOFF)
                    .finallyDo(interrupted -> feeder.resetSpeed());
            feederCommand = Commands.sequence(backoff, feedPhase);
        } else {
            feederCommand = feedPhase;
        }

        feederCommand.initialize();
    }

    @Override
    public void execute() {
        if (feederCommand != null) {
            feederCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        // Safety timeout: spinup wait (if any) + delay + feed duration
        double totalDuration = (requireAtSpeed ? AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT : 0)
                + spinupDelaySec + durationSec;
        if (timer.hasElapsed(totalDuration)) {
            return true;
        }
        
        if (feederCommand != null) {
            return feederCommand.isFinished();
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        double matchTime = DriverStation.getMatchTime();
        RobotLogger.log(String.format("[CmdShootForTime] t=%.2f Finished: interrupted=%b elapsed=%.2fs matchTime=%.2f",
                Timer.getFPGATimestamp(), interrupted, timer.get(), matchTime));
        if (feederCommand != null) {
            feederCommand.end(interrupted);
        }
        // Call reset directly; do not schedule - scheduling commands that require these
        // subsystems would cancel this command's parent sequence before it can proceed.
        feeder.resetSpeed();
        if (floor != null) {
            floor.resetSpeed();
        }
        shooter.resetSpeed();
    }

    /**
     * Factory method to create a command without requiring shooter at speed.
     * Runs feeder and floor (if non-null) to feed notes into the shooter.
     * Uses default spin-up delay ({@link AutoConstants#SHOOT_SPINUP_DELAY_BEFORE_FEED}).
     *
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor Optional floor subsystem; if non-null, runs in parallel with feeder to feed from conveyor
     * @param durationSec Duration to feed in seconds
     * @return A command that shoots for the specified duration
     */
    public static Command create(Shooter shooter, Feeder feeder, Floor floor, double durationSec) {
        return create(shooter, feeder, floor, durationSec, AutoConstants.SHOOT_SPINUP_DELAY_BEFORE_FEED);
    }

    /**
     * Factory method with configurable spin-up delay. Use for routines that need longer shooter
     * stabilization (e.g. ShooterAuto Left/Right at angle).
     *
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor Optional floor subsystem; if non-null, runs in parallel with feeder to feed from conveyor
     * @param durationSec Duration to feed in seconds
     * @param spinupDelaySec Delay (seconds) before feeder/floor start
     * @return A command that shoots for the specified duration
     */
    public static Command create(Shooter shooter, Feeder feeder, Floor floor, double durationSec, double spinupDelaySec) {
        return create(shooter, feeder, floor, durationSec, spinupDelaySec, null);
    }

    /**
     * Factory method with spin-up delay and optional per-motor shooter speeds. When shooterSpeedsSupplier is
     * non-null, runs the shooter at those speeds for the full duration (delay + feed). For a single
     * base RPM in auto use {@code () -> ShooterSpeeds.uniform(rpm)}.
     */
    public static Command create(Shooter shooter, Feeder feeder, Floor floor, double durationSec, double spinupDelaySec, Supplier<ShooterSpeeds> shooterSpeedsSupplier) {
        return new CmdShootForTime(shooter, feeder, floor, durationSec, spinupDelaySec, null, shooterSpeedsSupplier, false, false, null, null, 0.0);
    }

    /**
     * Factory method (no floor). Use {@link #create(Shooter, Feeder, Floor, double)} when floor is available.
     */
    public static Command create(Shooter shooter, Feeder feeder, double durationSec) {
        return create(shooter, feeder, null, durationSec);
    }
}
