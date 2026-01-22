package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to hold climber and stop drivetrain until auto end.
 * 
 * <p>This command holds the climber at its current position and stops the drivetrain.
 * It runs indefinitely until the autonomous period ends or the command is interrupted.
 * This is typically used at the end of a climber auto routine to maintain the climb
 * position for the remainder of autonomous.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Command is interrupted (runs indefinitely otherwise)</li>
 * </ul>
 * 
 * @param climber The climber subsystem
 * @param drivetrain The drivetrain subsystem
 */
public class CmdHoldClimbUntilEnd extends Command {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private Command holdCommand;

    /**
     * Creates a new CmdHoldClimbUntilEnd command.
     * 
     * @param climber The climber subsystem
     * @param drivetrain The drivetrain subsystem
     * @throws NullPointerException if climber or drivetrain is null
     */
    public CmdHoldClimbUntilEnd(Climber climber, CommandSwerveDrivetrain drivetrain) {
        this.climber = Objects.requireNonNull(climber, "climber cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");

        addRequirements(climber, drivetrain);
    }

    @Override
    public void initialize() {
        // Stop drivetrain
        drivetrain.stop();
        
        // Hold climber at current position
        // Use moveToArbitraryPositionCommand to hold current position
        double currentPosition = climber.getPosition();
        holdCommand = climber.moveToArbitraryPositionCommand(() -> currentPosition)
                .withName("CmdHoldClimbUntilEnd-hold");
        holdCommand.initialize();
    }

    @Override
    public void execute() {
        // Keep drivetrain stopped
        drivetrain.stop();
        
        // Continue holding climber
        if (holdCommand != null) {
            holdCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        // Runs indefinitely until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (holdCommand != null) {
            holdCommand.end(interrupted);
        }
    }
}
