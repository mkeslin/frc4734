package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/**
 * Command to wait until the shooter reaches target speed.
 * 
 * <p>This command polls the shooter's current speed and waits until it is within
 * tolerance of the target speed. It is typically used after CmdShooterSpinUp to
 * ensure the shooter is ready before feeding.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Shooter speed is within rpmTol of target speed</li>
 *   <li>Timeout expires</li>
 * </ul>
 * 
 * @param shooter The shooter subsystem
 * @param targetRpmSupplier Supplier of the target RPM
 * @param rpmTol RPM tolerance
 * @param timeoutSec Maximum time to wait
 */
public class CmdWaitShooterAtSpeed extends Command {
    private final Shooter shooter;
    private final Supplier<Double> targetRpmSupplier;
    private final double rpmTol;
    private final double timeoutSec;
    private final Timer timer = new Timer();

    /**
     * Creates a new CmdWaitShooterAtSpeed command.
     * 
     * @param shooter The shooter subsystem
     * @param targetRpmSupplier Supplier of the target RPM
     * @param rpmTol RPM tolerance
     * @param timeoutSec Maximum time to wait
     * @throws NullPointerException if shooter or targetRpmSupplier is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdWaitShooterAtSpeed(
            Shooter shooter,
            Supplier<Double> targetRpmSupplier,
            double rpmTol,
            double timeoutSec) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.targetRpmSupplier = Objects.requireNonNull(targetRpmSupplier, "targetRpmSupplier cannot be null");
        this.rpmTol = rpmTol;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Command just waits - no action needed
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }

        double currentSpeed = shooter.getSpeed();
        double targetSpeed = targetRpmSupplier.get();
        double speedError = Math.abs(currentSpeed - targetSpeed);
        
        return speedError <= rpmTol;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    /**
     * Factory method to create a command with default timeout.
     * 
     * @param shooter The shooter subsystem
     * @param targetRpmSupplier Supplier of the target RPM
     * @param rpmTol RPM tolerance
     * @return A command that waits for shooter at speed with default timeout
     */
    public static Command create(Shooter shooter, Supplier<Double> targetRpmSupplier, double rpmTol) {
        return new CmdWaitShooterAtSpeed(
                shooter,
                targetRpmSupplier,
                rpmTol,
                AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT);
    }
}
