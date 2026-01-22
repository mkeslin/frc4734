package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

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
 * @param rpmSupplier Supplier of the target RPM
 */
public class CmdShooterSpinUp extends Command {
    private final Shooter shooter;
    private final Supplier<Double> rpmSupplier;
    private Command spinUpCommand;

    /**
     * Creates a new CmdShooterSpinUp command.
     * 
     * @param shooter The shooter subsystem
     * @param rpmSupplier Supplier of the target RPM
     * @throws NullPointerException if shooter or rpmSupplier is null
     */
    public CmdShooterSpinUp(Shooter shooter, Supplier<Double> rpmSupplier) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.rpmSupplier = Objects.requireNonNull(rpmSupplier, "rpmSupplier cannot be null");

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        double targetRpm = rpmSupplier.get();
        spinUpCommand = shooter.moveToArbitrarySpeedCommand(() -> targetRpm);
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
