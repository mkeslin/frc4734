package frc.robot.Auto.commands;

import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_BACKOFF;
import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_PULSE_DELAY_SEC;
import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_PULSE_ON_SEC;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FeederConstants.FeederSpeed;
import frc.robot.Constants.FloorConstants.ConveyorSpeed;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;

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
    private final Supplier<Double> shooterRpmSupplierForRun;
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
            boolean requireAtSpeed,
            boolean useFeederBackoff,
            Supplier<Double> targetRpmSupplier,
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
        this.requireAtSpeed = requireAtSpeed;
        this.useFeederBackoff = useFeederBackoff;
        this.targetRpmSupplier = requireAtSpeed
                ? Objects.requireNonNull(targetRpmSupplier, "targetRpmSupplier cannot be null if requireAtSpeed is true")
                : targetRpmSupplier;
        this.rpmTol = rpmTol;

        addRequirements(shooter, feeder);
        if (floor != null) {
            addRequirements(floor);
        }
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
            Command waitCommand = new CmdWaitShooterAtSpeed(
                    shooter,
                    targetRpmSupplier,
                    rpmTol,
                    AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT);
            feedPhase = Commands.sequence(waitCommand, feedPhase);
        }

        // Extra spin-up time before feeder starts so shooter stabilizes
        feedPhase = Commands.sequence(
                Commands.waitSeconds(spinupDelaySec),
                feedPhase);

        // When shooterRpmSupplierForRun is set, run shooter in parallel with feed phase (inside this
        // command to avoid ParallelCommandGroup subsystem conflict: shooter vs feeder+floor only)
        if (shooterRpmSupplierForRun != null) {
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
        if (feederCommand != null) {
            feederCommand.end(interrupted);
        }
        var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
        scheduler.schedule(feeder.resetSpeedCommand());
        if (floor != null) {
            scheduler.schedule(floor.resetSpeedCommand());
        }
        scheduler.schedule(new CmdStopShooter(shooter));
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
     * Factory method with spin-up delay and optional shooter RPM. When shooterRpmSupplier is
     * non-null, runs the shooter at that speed for the full duration (delay + feed). Use when
     * the shoot step is not preceded by a spin-up command (e.g. ShooterAuto Left/Right).
     *
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor Optional floor subsystem; if non-null, runs in parallel with feeder
     * @param durationSec Duration to feed in seconds
     * @param spinupDelaySec Delay (seconds) before feeder/floor start
     * @param shooterRpmSupplier Optional; when non-null, runs shooter at this speed for full duration
     * @return A command that shoots for the specified duration
     */
    public static Command create(Shooter shooter, Feeder feeder, Floor floor, double durationSec, double spinupDelaySec, Supplier<Double> shooterRpmSupplier) {
        return new CmdShootForTime(shooter, feeder, floor, durationSec, spinupDelaySec, shooterRpmSupplier, false, false, null, 0.0);
    }

    /**
     * Factory method (no floor). Use {@link #create(Shooter, Feeder, Floor, double)} when floor is available.
     */
    public static Command create(Shooter shooter, Feeder feeder, double durationSec) {
        return create(shooter, feeder, null, durationSec);
    }
}
