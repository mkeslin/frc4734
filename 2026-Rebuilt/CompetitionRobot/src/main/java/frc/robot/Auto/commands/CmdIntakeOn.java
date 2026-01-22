package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DeployableIntake;

/**
 * Command to turn the intake on.
 * 
 * <p>This command starts the intake motor at the specified speed. If no speed
 * supplier is provided, it uses a default forward speed.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Intake is started (completes immediately, non-blocking)</li>
 * </ul>
 * 
 * @param intake The intake subsystem
 * @param speedSupplier Supplier of intake speed (optional, defaults to forward)
 */
public class CmdIntakeOn extends Command {
    private final DeployableIntake intake;
    private final Supplier<Double> speedSupplier;
    private Command intakeCommand;

    /**
     * Creates a new CmdIntakeOn command.
     * 
     * @param intake The intake subsystem
     * @param speedSupplier Supplier of intake speed (can be null for default)
     * @throws NullPointerException if intake is null
     */
    public CmdIntakeOn(DeployableIntake intake, Supplier<Double> speedSupplier) {
        this.intake = Objects.requireNonNull(intake, "intake cannot be null");
        this.speedSupplier = speedSupplier;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        double speed = speedSupplier != null ? speedSupplier.get() : 0.5; // Default forward speed
        // TODO: Replace with actual intake speed constant from IntakeConstants
        intakeCommand = intake.moveToArbitraryIntakeSpeedCommand(() -> speed);
        intakeCommand.initialize();
    }

    @Override
    public void execute() {
        if (intakeCommand != null) {
            intakeCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        // Non-blocking - let intake run continuously
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Don't stop intake on end - let it continue running
        // The intake will be stopped by a separate stop command if needed
    }

    /**
     * Factory method to create a command with default forward speed.
     * 
     * @param intake The intake subsystem
     * @return A command that turns intake on with default speed
     */
    public static Command create(DeployableIntake intake) {
        return new CmdIntakeOn(intake, null);
    }
}
