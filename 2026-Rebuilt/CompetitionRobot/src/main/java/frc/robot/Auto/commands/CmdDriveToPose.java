package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to drive to a target pose using pathfinding.
 * 
 * <p>This command uses the drivetrain's pathfinding capabilities to navigate
 * to a target pose. It completes when the robot reaches the pose within
 * specified tolerances or when the timeout expires.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Robot is at target pose (within xyTol and rotTol)</li>
 *   <li>Timeout expires</li>
 * </ul>
 * 
 * @param drivetrain The drivetrain subsystem
 * @param targetPoseSupplier Supplier of the target pose
 * @param xyTol XY position tolerance in meters
 * @param rotTol Rotation tolerance in radians
 * @param timeoutSec Maximum time to reach the pose
 */
public class CmdDriveToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final double xyTol;
    private final double rotTol;
    private final double timeoutSec;
    private final Timer timer = new Timer();
    private Command pathfindingCommand;

    /**
     * Creates a new CmdDriveToPose command.
     * 
     * @param drivetrain The drivetrain subsystem
     * @param targetPoseSupplier Supplier of the target pose
     * @param xyTol XY position tolerance in meters
     * @param rotTol Rotation tolerance in radians
     * @param timeoutSec Maximum time to reach the pose
     * @throws NullPointerException if drivetrain or targetPoseSupplier is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdDriveToPose(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            double xyTol,
            double rotTol,
            double timeoutSec) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.targetPoseSupplier = Objects.requireNonNull(targetPoseSupplier, "targetPoseSupplier cannot be null");
        this.xyTol = xyTol;
        this.rotTol = rotTol;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Pose2d targetPose = targetPoseSupplier.get();
        if (targetPose != null) {
            pathfindingCommand = drivetrain.moveToPose(targetPose)
                    .withTimeout(timeoutSec)
                    .withName("CmdDriveToPose");
            pathfindingCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (pathfindingCommand != null) {
            pathfindingCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }

        if (pathfindingCommand != null && pathfindingCommand.isFinished()) {
            // Check if we're actually at the pose (pathfinding may finish early)
            Pose2d currentPose = drivetrain.getPose();
            Pose2d targetPose = targetPoseSupplier.get();
            if (targetPose != null && atPose(currentPose, targetPose, xyTol, rotTol)) {
                return true;
            }
        }

        // Also check directly if we're at pose (in case pathfinding finished but we're close)
        Pose2d currentPose = drivetrain.getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        if (targetPose != null && atPose(currentPose, targetPose, xyTol, rotTol)) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (pathfindingCommand != null) {
            pathfindingCommand.end(interrupted);
        }
    }

    /**
     * Checks if the current pose is within tolerance of the target pose.
     * 
     * @param currentPose The current robot pose
     * @param targetPose The target pose
     * @param xyTol XY position tolerance in meters
     * @param rotTol Rotation tolerance in radians
     * @return true if within tolerance, false otherwise
     */
    private boolean atPose(Pose2d currentPose, Pose2d targetPose, double xyTol, double rotTol) {
        Translation2d translationError = currentPose.getTranslation().minus(targetPose.getTranslation());
        double xyError = translationError.getNorm();
        double rotError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        
        return xyError <= xyTol && rotError <= rotTol;
    }

    /**
     * Factory method to create a command with default tolerances and timeout.
     * 
     * @param drivetrain The drivetrain subsystem
     * @param targetPoseSupplier Supplier of the target pose
     * @return A command that drives to the pose with default settings
     */
    public static Command create(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        return new CmdDriveToPose(
                drivetrain,
                targetPoseSupplier,
                AutoConstants.DEFAULT_XY_TOLERANCE,
                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                AutoConstants.DEFAULT_POSE_TIMEOUT);
    }
}
