package frc.robot.Auto.commands;

import java.util.Objects;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Logging.RobotLogger;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to follow a PathPlanner path.
 * 
 * <p>This command loads a path from the PathPlanner path files and follows it
 * using the AutoBuilder. The path is loaded by name from the deploy directory.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Path is successfully completed</li>
 *   <li>Timeout expires</li>
 *   <li>Path file is missing or invalid (returns empty command)</li>
 * </ul>
 * 
 * @param pathName The name of the path file (without .path extension)
 * @param timeoutSec Maximum time to follow the path
 * @param drivetrain The drivetrain subsystem
 */
public class CmdFollowPath extends Command {
    private final String pathName;
    private final double timeoutSec;
    private final CommandSwerveDrivetrain drivetrain;
    private Command pathCommand;

    /**
     * Creates a new CmdFollowPath command.
     * 
     * @param pathName The name of the path file (without .path extension)
     * @param timeoutSec Maximum time to follow the path
     * @param drivetrain The drivetrain subsystem
     * @throws NullPointerException if pathName or drivetrain is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdFollowPath(String pathName, double timeoutSec, CommandSwerveDrivetrain drivetrain) {
        this.pathName = Objects.requireNonNull(pathName, "pathName cannot be null");
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception exception) {
            String errorMsg = String.format("[CmdFollowPath] Failed to load path '%s': %s", 
                    pathName, exception.getMessage());
            RobotLogger.logError(errorMsg);
            DriverStation.reportError(errorMsg, false);
        }

        if (path != null) {
            pathCommand = drivetrain.followPathCommand(path)
                    .withTimeout(timeoutSec)
                    .withName("CmdFollowPath-" + pathName);
            pathCommand.initialize();
        } else {
            // Path failed to load - command will finish immediately
            pathCommand = Commands.none();
        }
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (pathCommand == null) {
            return true; // Path failed to load
        }
        return pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }
    }

    /**
     * Factory method to create a command with default timeout.
     * 
     * @param pathName The name of the path file
     * @param drivetrain The drivetrain subsystem
     * @return A command that follows the path with default timeout
     */
    public static Command create(String pathName, CommandSwerveDrivetrain drivetrain) {
        return new CmdFollowPath(pathName, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain);
    }
}
