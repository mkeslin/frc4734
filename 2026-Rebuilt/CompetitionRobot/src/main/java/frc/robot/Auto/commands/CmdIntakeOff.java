package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DeployableIntake;

/**
 * Command to turn the intake off.
 * 
 * <p>This command stops the intake motor by resetting its speed to zero.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Intake stop command completes (immediate)</li>
 * </ul>
 * 
 * @param intake The intake subsystem
 */
public class CmdIntakeOff extends Command {
    private final DeployableIntake intake;

    /**
     * Creates a new CmdIntakeOff command.
     * 
     * @param intake The intake subsystem
     * @throws NullPointerException if intake is null
     */
    public CmdIntakeOff(DeployableIntake intake) {
        this.intake = Objects.requireNonNull(intake, "intake cannot be null");
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(intake.resetIntakeSpeedCommand());
    }

    @Override
    public boolean isFinished() {
        // Completes immediately after scheduling stop
        return true;
    }
}
