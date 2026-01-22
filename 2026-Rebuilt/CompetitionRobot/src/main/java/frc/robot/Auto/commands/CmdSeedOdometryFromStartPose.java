package frc.robot.Auto.commands;

import java.util.Map;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to seed odometry from a predefined start pose.
 * 
 * <p>This command resets the drivetrain's pose estimator to a known starting position
 * based on the selected start pose ID. This is typically the first command in an
 * autonomous routine to establish accurate field-relative positioning.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Odometry is successfully reset to the start pose</li>
 *   <li>Timeout expires (safety fallback)</li>
 * </ul>
 * 
 * @param startPoseId The identifier for the starting position
 * @param startPoses Map of start pose IDs to their corresponding Pose2d values
 * @param drivetrain The drivetrain subsystem to reset
 */
public class CmdSeedOdometryFromStartPose extends Command {
    private final StartPoseId startPoseId;
    private final Map<StartPoseId, Pose2d> startPoses;
    private final CommandSwerveDrivetrain drivetrain;

    /**
     * Creates a new CmdSeedOdometryFromStartPose command.
     * 
     * @param startPoseId The identifier for the starting position
     * @param startPoses Map of start pose IDs to their corresponding Pose2d values
     * @param drivetrain The drivetrain subsystem to reset
     * @throws NullPointerException if any parameter is null
     */
    public CmdSeedOdometryFromStartPose(
            StartPoseId startPoseId,
            Map<StartPoseId, Pose2d> startPoses,
            CommandSwerveDrivetrain drivetrain) {
        this.startPoseId = Objects.requireNonNull(startPoseId, "startPoseId cannot be null");
        this.startPoses = Objects.requireNonNull(startPoses, "startPoses cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d startPose = startPoses.get(startPoseId);
        if (startPose != null) {
            drivetrain.resetPoseEstimator(startPose);
        }
    }

    @Override
    public boolean isFinished() {
        // Command completes immediately after resetting odometry
        return true;
    }

    /**
     * Factory method to create a command with timeout protection.
     * 
     * @param startPoseId The identifier for the starting position
     * @param startPoses Map of start pose IDs to their corresponding Pose2d values
     * @param drivetrain The drivetrain subsystem to reset
     * @return A command that seeds odometry with timeout protection
     */
    public static Command create(StartPoseId startPoseId, Map<StartPoseId, Pose2d> startPoses,
            CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
                new CmdSeedOdometryFromStartPose(startPoseId, startPoses, drivetrain))
                .withTimeout(AutoConstants.DEFAULT_ODOMETRY_SEED_TIMEOUT)
                .withName("CmdSeedOdometryFromStartPose");
    }
}
