package frc.robot.Auto.commands;

import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_BACKOFF;
import static frc.robot.Constants.CommandConstants.SHOOT_FLOOR_DELAY;

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
    private final boolean requireAtSpeed;
    private final boolean useFeederBackoff;
    private final Supplier<Double> targetRpmSupplier;
    private final double rpmTol;
    private final Timer timer = new Timer();
    private Command feederCommand;

    /**
     * Full constructor with optional feeder backoff and optional floor.
     *
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param floor Optional floor subsystem; if non-null, floor runs after {@code SHOOT_FLOOR_DELAY} and is stopped in end()
     * @param durationSec Duration to feed in seconds
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
        this(shooter, feeder, null, durationSec, requireAtSpeed, false, targetRpmSupplier, rpmTol);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        Command feedPhase = feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value)
                .withTimeout(durationSec)
                .withName("CmdShootForTime-feeder");

        if (floor != null) {
            Command floorPhase = Commands.waitSeconds(SHOOT_FLOOR_DELAY)
                    .andThen(floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value).withTimeout(Math.max(0, durationSec - SHOOT_FLOOR_DELAY)));
            feedPhase = new ParallelCommandGroup(feedPhase, floorPhase);
        }

        if (requireAtSpeed) {
            Command waitCommand = new CmdWaitShooterAtSpeed(
                    shooter,
                    targetRpmSupplier,
                    rpmTol,
                    AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT);
            feedPhase = Commands.sequence(waitCommand, feedPhase);
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
        if (timer.hasElapsed(durationSec)) {
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
    }

    /**
     * Factory method to create a command without requiring shooter at speed.
     * 
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param durationSec Duration to feed in seconds
     * @return A command that shoots for the specified duration
     */
    public static Command create(Shooter shooter, Feeder feeder, double durationSec) {
        return new CmdShootForTime(shooter, feeder, durationSec, false, null, 0.0);
    }
}
