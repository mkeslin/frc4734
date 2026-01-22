package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Subsystems.Climber;

/**
 * Command to execute L1 climb sequence.
 * 
 * <p>This command moves the climber to the CLIMB position to execute an L1 climb.
 * It includes timeout protection and checks if the climber has reached the
 * qualified position if such a method exists.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Climber reaches CLIMB position</li>
 *   <li>Climber is L1 qualified (if method exists)</li>
 *   <li>Timeout expires</li>
 * </ul>
 * 
 * @param climber The climber subsystem
 * @param timeoutSec Maximum time to complete climb
 */
public class CmdClimbL1 extends Command {
    private final Climber climber;
    private final double timeoutSec;
    private final Timer timer = new Timer();
    private Command climbCommand;

    /**
     * Creates a new CmdClimbL1 command.
     * 
     * @param climber The climber subsystem
     * @param timeoutSec Maximum time to complete climb
     * @throws NullPointerException if climber is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdClimbL1(Climber climber, double timeoutSec) {
        this.climber = Objects.requireNonNull(climber, "climber cannot be null");
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        // Move climber to CLIMB position
        climbCommand = climber.moveToSetPositionCommand(() -> ClimberPosition.CLIMB)
                .withTimeout(timeoutSec)
                .withName("CmdClimbL1");
        climbCommand.initialize();
    }

    @Override
    public void execute() {
        if (climbCommand != null) {
            climbCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }

        // Check if climber has reached CLIMB position
        double currentPosition = climber.getPosition();
        double targetPosition = ClimberPosition.CLIMB.value;
        double positionError = Math.abs(currentPosition - targetPosition);
        
        if (positionError < 1.0) { // Within 1 rotation
            // TODO: Check climber.isL1Qualified() if method exists
            // For now, just check position
            return true;
        }

        if (climbCommand != null && climbCommand.isFinished()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (climbCommand != null) {
            climbCommand.end(interrupted);
        }
    }

    /**
     * Factory method to create a command with default timeout.
     * 
     * @param climber The climber subsystem
     * @return A command that climbs L1 with default timeout
     */
    public static Command create(Climber climber) {
        return new CmdClimbL1(climber, AutoConstants.DEFAULT_CLIMB_TIMEOUT);
    }
}
