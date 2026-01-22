package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Feeder;
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
    private final double durationSec;
    private final boolean requireAtSpeed;
    private final Supplier<Double> targetRpmSupplier;
    private final double rpmTol;
    private final Timer timer = new Timer();
    private Command feederCommand;

    /**
     * Creates a new CmdShootForTime command.
     * 
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param durationSec Duration to feed in seconds
     * @param requireAtSpeed If true, wait for shooter at speed before feeding
     * @param targetRpmSupplier Supplier of target RPM (required if requireAtSpeed is true)
     * @param rpmTol RPM tolerance (required if requireAtSpeed is true)
     * @throws NullPointerException if shooter or feeder is null
     * @throws IllegalArgumentException if durationSec is less than or equal to 0
     */
    public CmdShootForTime(
            Shooter shooter,
            Feeder feeder,
            double durationSec,
            boolean requireAtSpeed,
            Supplier<Double> targetRpmSupplier,
            double rpmTol) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.feeder = Objects.requireNonNull(feeder, "feeder cannot be null");
        if (durationSec <= 0) {
            throw new IllegalArgumentException("durationSec must be greater than 0, got: " + durationSec);
        }
        this.durationSec = durationSec;
        this.requireAtSpeed = requireAtSpeed;
        this.targetRpmSupplier = requireAtSpeed 
                ? Objects.requireNonNull(targetRpmSupplier, "targetRpmSupplier cannot be null if requireAtSpeed is true")
                : targetRpmSupplier;
        this.rpmTol = rpmTol;

        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // Create feeder command (run at forward speed for duration)
        // TODO: Replace with actual feeder forward speed constant
        double feederSpeed = 0.5; // Placeholder - should come from FeederConstants
        feederCommand = feeder.moveToArbitrarySpeedCommand(() -> feederSpeed)
                .withTimeout(durationSec)
                .withName("CmdShootForTime-feeder");
        
        if (requireAtSpeed) {
            // Wait for shooter at speed first
            Command waitCommand = new CmdWaitShooterAtSpeed(
                    shooter,
                    targetRpmSupplier,
                    rpmTol,
                    AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT);
            
            feederCommand = Commands.sequence(waitCommand, feederCommand);
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
        // Stop feeder on end
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(feeder.resetSpeedCommand());
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
