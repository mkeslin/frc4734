package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.ShooterSpeeds;

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
 * @param targetRpmSupplier Supplier of the target RPM (single value)
 * @param rpmTol RPM tolerance
 * @param timeoutSec Maximum time to wait
 */
public class CmdWaitShooterAtSpeed extends Command {
    private final Shooter shooter;
    private final Supplier<ShooterSpeeds> targetSpeedsSupplier;
    private final double rpmTol;
    private final double timeoutSec;
    private final Timer timer = new Timer();

    /**
     * Creates a new CmdWaitShooterAtSpeed command with per-motor speeds.
     * For a single base RPM in auto, use {@code () -> ShooterSpeeds.uniform(rpm)}.
     *
     * @param shooter The shooter subsystem
     * @param targetSpeedsSupplier Supplier of per-motor speeds (left, center, right RPS)
     * @param rpmTol RPM tolerance
     * @param timeoutSec Maximum time to wait
     * @throws NullPointerException if shooter or targetSpeedsSupplier is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdWaitShooterAtSpeed(
            Shooter shooter,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
            double rpmTol,
            double timeoutSec) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.targetSpeedsSupplier = Objects.requireNonNull(targetSpeedsSupplier, "targetSpeedsSupplier cannot be null");
        this.rpmTol = rpmTol;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
        // No subsystem requirement: only reads shooter state; does not control it.
        // Allows running in parallel with CmdShooterSpinUp (which does require Shooter).
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
        return shooter.isAtSpeed(targetSpeedsSupplier.get(), rpmTol);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    /**
     * Factory method to create a command with default timeout.
     * For a single base RPM in auto, use {@code () -> ShooterSpeeds.uniform(rpm)}.
     */
    public static Command create(Shooter shooter, Supplier<ShooterSpeeds> targetSpeedsSupplier, double rpmTol) {
        return new CmdWaitShooterAtSpeed(
                shooter,
                targetSpeedsSupplier,
                rpmTol,
                AutoConstants.DEFAULT_SHOOTER_SPINUP_TIMEOUT);
    }
}
