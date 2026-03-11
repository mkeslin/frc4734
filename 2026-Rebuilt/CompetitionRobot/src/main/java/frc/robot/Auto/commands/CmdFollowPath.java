package frc.robot.Auto.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Logging.RobotLogger;
import frc.robot.PathPlanner.AllianceUtils;
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
    private final double maxVelocityMps;
    private final double maxAccelMps2;
    private Command pathCommand;

    /**
     * Creates a new CmdFollowPath command with default velocity constraints.
     *
     * @param pathName The name of the path file (without .path extension)
     * @param timeoutSec Maximum time to follow the path
     * @param drivetrain The drivetrain subsystem
     * @throws NullPointerException if pathName or drivetrain is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdFollowPath(String pathName, double timeoutSec, CommandSwerveDrivetrain drivetrain) {
        this(pathName, timeoutSec, drivetrain, AutoConstants.AUTO_PATH_MAX_VELOCITY_MPS, AutoConstants.AUTO_PATH_MAX_ACCELERATION_MPS2);
    }

    /**
     * Creates a new CmdFollowPath command with custom velocity constraints.
     *
     * @param pathName The name of the path file (without .path extension)
     * @param timeoutSec Maximum time to follow the path
     * @param drivetrain The drivetrain subsystem
     * @param maxVelocityMps Maximum velocity (m/s) for path following
     * @param maxAccelMps2 Maximum acceleration (m/s²) for path following
     * @throws NullPointerException if pathName or drivetrain is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdFollowPath(String pathName, double timeoutSec, CommandSwerveDrivetrain drivetrain, double maxVelocityMps, double maxAccelMps2) {
        this.pathName = Objects.requireNonNull(pathName, "pathName cannot be null");
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.maxVelocityMps = maxVelocityMps;
        this.maxAccelMps2 = maxAccelMps2;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        System.out.println(String.format("[CmdFollowPath] initialize: pathName='%s' alliance=%s", pathName, alliance));

        // Try alliance-specific path first (e.g. C_StartToShot_Red.path); if present, use without flipping.
        String alliancePathName = pathName + "_" + (alliance == Alliance.Red ? "Red" : "Blue");
        PathPlannerPath path = null;
        String loadedFrom = null;
        try {
            path = PathPlannerPath.fromPathFile(alliancePathName);
            loadedFrom = alliancePathName;
            System.out.println(String.format("[CmdFollowPath] Loaded alliance-specific path '%s'", alliancePathName));
        } catch (Exception e) {
            try {
                path = PathPlannerPath.fromPathFile(pathName);
                loadedFrom = pathName;
                System.out.println(String.format("[CmdFollowPath] Loaded base path '%s' (alliance-specific '%s' not found)", pathName, alliancePathName));
            } catch (Exception exception) {
                String errorMsg = String.format("[CmdFollowPath] Failed to load path '%s': %s",
                        pathName, exception.getMessage());
                RobotLogger.logError(errorMsg);
                DriverStation.reportError(errorMsg, false);
                System.out.println(errorMsg);
            }
        }

        if (path != null) {
            var points = path.getAllPathPoints();
            if (!points.isEmpty()) {
                Translation2d first = points.get(0).position;
                Translation2d last = points.get(points.size() - 1).position;
                System.out.println(String.format("[CmdFollowPath] Path '%s' points: first=(%.2f, %.2f) last=(%.2f, %.2f)",
                        loadedFrom, first.getX(), first.getY(), last.getX(), last.getY()));
            }
            var robotPose = drivetrain.getState().Pose;
            System.out.println(String.format("[CmdFollowPath] Robot odometry pose: (%.2f, %.2f) rot=%.1f deg",
                    robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees()));
            if (alliance == Alliance.Red) {
                var blueEquiv = AllianceUtils.redToBlue(robotPose);
                var poseForPathPlanner = new Pose2d(
                        AllianceUtils.flipPositionForPathPlanner(blueEquiv.getTranslation()),
                        AllianceUtils.flipRotationForPathPlanner(blueEquiv.getRotation()));
                System.out.println(String.format("[CmdFollowPath] Pose supplier (red->blue frame): (%.2f, %.2f) rot=%.1f deg",
                        poseForPathPlanner.getX(), poseForPathPlanner.getY(), poseForPathPlanner.getRotation().getDegrees()));
            }

            var orig = path.getGlobalConstraints();
            PathConstraints constraints = new PathConstraints(
                    maxVelocityMps,
                    maxAccelMps2,
                    orig.maxAngularVelocityRadPerSec(),
                    orig.maxAngularAccelerationRadPerSecSq(),
                    orig.nominalVoltageVolts());
            PathPlannerPath pathToFollow;
            if (alliance == Alliance.Red && loadedFrom.equals(pathName)) {
                // No alliance-specific path; manually flip base path for red.
                List<PathPoint> flippedPoints = new ArrayList<>();
                for (PathPoint pt : path.getAllPathPoints()) {
                    Translation2d flippedPos = AllianceUtils.flipPositionForPathPlanner(pt.position);
                    PathPoint flipped = pt.rotationTarget != null
                            ? new PathPoint(flippedPos,
                                    new RotationTarget(pt.rotationTarget.position(),
                                            AllianceUtils.flipRotationForPathPlanner(pt.rotationTarget.rotation())),
                                    pt.constraints)
                            : new PathPoint(flippedPos);
                    flippedPoints.add(flipped);
                }
                GoalEndState origGoal = path.getGoalEndState();
                GoalEndState flippedGoal = new GoalEndState(
                        origGoal.velocityMPS(),
                        AllianceUtils.flipRotationForPathPlanner(origGoal.rotation()));
                pathToFollow = PathPlannerPath.fromPathPoints(flippedPoints, constraints, flippedGoal);
                pathToFollow.preventFlipping = true;
                var fp = flippedPoints.get(0).position;
                var lp = flippedPoints.get(flippedPoints.size() - 1).position;
                System.out.println(String.format("[CmdFollowPath] Flipped for red: first=(%.2f, %.2f) last=(%.2f, %.2f)",
                        fp.getX(), fp.getY(), lp.getX(), lp.getY()));
                RobotLogger.log(String.format("[CmdFollowPath] Flipped path '%s' for red alliance", pathName));
            } else {
                pathToFollow = PathPlannerPath.fromPathPoints(
                        path.getAllPathPoints(),
                        constraints,
                        path.getGoalEndState());
                if (alliance == Alliance.Red) {
                    pathToFollow.preventFlipping = true; // Alliance-specific path already in red coords
                }
            }
            pathCommand = drivetrain.followPathCommand(pathToFollow)
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
