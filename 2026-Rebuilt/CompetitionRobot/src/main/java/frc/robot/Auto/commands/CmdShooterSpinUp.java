package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logging.RobotLogger;
import frc.robot.Subsystems.Shooter;
import frc.robot.ShooterSpeeds;

/**
 * Command to spin up the shooter to a target RPM (non-blocking).
 * 
 * <p>This command starts the shooter spinning to the target RPM but does not wait
 * for it to reach speed. It is typically used in parallel with other commands
 * (e.g., following a path) to allow the shooter to spin up while the robot moves.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Shooter spin-up command is started (completes immediately)</li>
 * </ul>
 * 
 * @param shooter The shooter subsystem
 * @param rpmSupplier Supplier of the target RPM (single value) or per-motor speeds
 */
public class CmdShooterSpinUp extends Command {
    private final Shooter shooter;
    private final Supplier<ShooterSpeeds> speedsSupplier;
    private Command spinUpCommand;

    /**
     * Creates a new CmdShooterSpinUp command with per-motor speeds.
     * For a single base RPM in auto, use {@code () -> ShooterSpeeds.uniform(rpm)}.
     *
     * @param shooter The shooter subsystem
     * @param speedsSupplier Supplier of per-motor speeds (left, center, right RPS)
     * @throws NullPointerException if shooter or speedsSupplier is null
     */
    public CmdShooterSpinUp(Shooter shooter, Supplier<ShooterSpeeds> speedsSupplier) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.speedsSupplier = Objects.requireNonNull(speedsSupplier, "speedsSupplier cannot be null");
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        ShooterSpeeds s = speedsSupplier.get();
        RobotLogger.log(String.format("[CmdShooterSpinUp] Target shooter speeds: L=%.1f C=%.1f R=%.1f RPS", s.leftRps(), s.centerRps(), s.rightRps()));
        spinUpCommand = shooter.moveToTripleSpeedCommand(speedsSupplier);
        spinUpCommand.initialize();
    }

    @Override
    public void execute() {
        if (spinUpCommand != null) {
            spinUpCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        // Non-blocking - let the spin-up command run in background
        // This command itself never finishes, but the spin-up command will continue
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Don't stop shooter on end - let it continue spinning
        // The shooter will be stopped by a separate stop command if needed
    }
}
