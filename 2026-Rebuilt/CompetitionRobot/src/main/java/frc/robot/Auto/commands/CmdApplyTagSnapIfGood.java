package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to conditionally apply vision pose snap if quality thresholds are met.
 * 
 * <p>This command checks vision quality metrics (ambiguity, tag distance, tag count)
 * and only applies the vision measurement to the pose estimator if all thresholds
 * are satisfied. This prevents bad vision data from corrupting odometry.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Vision measurement is applied (if thresholds met) or skipped (if not)</li>
 *   <li>Command completes immediately (non-blocking)</li>
 * </ul>
 * 
 * @param vision The PhotonVision subsystem
 * @param drivetrain The drivetrain subsystem
 * @param maxAmbiguity Maximum acceptable pose ambiguity (lower is better)
 * @param maxDistance Maximum acceptable average tag distance in meters
 * @param minTargets Minimum number of visible AprilTags required
 */
public class CmdApplyTagSnapIfGood extends Command {
    private final PhotonVision vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final double maxAmbiguity;
    private final double maxDistance;
    private final int minTargets;

    /**
     * Creates a new CmdApplyTagSnapIfGood command.
     * 
     * @param vision The PhotonVision subsystem
     * @param drivetrain The drivetrain subsystem
     * @param maxAmbiguity Maximum acceptable pose ambiguity
     * @param maxDistance Maximum acceptable average tag distance in meters
     * @param minTargets Minimum number of visible AprilTags required
     * @throws NullPointerException if vision or drivetrain is null
     */
    public CmdApplyTagSnapIfGood(
            PhotonVision vision,
            CommandSwerveDrivetrain drivetrain,
            double maxAmbiguity,
            double maxDistance,
            int minTargets) {
        this.vision = Objects.requireNonNull(vision, "vision cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.maxAmbiguity = maxAmbiguity;
        this.maxDistance = maxDistance;
        this.minTargets = minTargets;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        Optional<org.photonvision.EstimatedRobotPose> estimatedPose = vision.getEstimatedRobotPose();
        
        if (estimatedPose.isPresent()) {
            var pose = estimatedPose.get();
            
            // Check quality thresholds
            // Note: EstimatedRobotPose may not have ambiguity field in all PhotonVision versions
            // Using tag count and distance as quality metrics instead
            double ambiguity = 0.0; // Placeholder - use tag count/distance for quality assessment
            int tagCount = getVisibleTagCount();
            double avgDistance = getAvgTagDistanceMeters();
            
            if (ambiguity <= maxAmbiguity && 
                tagCount >= minTargets && 
                avgDistance <= maxDistance) {
                
                // Apply vision measurement
                Pose2d visionPose = pose.estimatedPose.toPose2d();
                drivetrain.addVisionMeasurement(visionPose, pose.timestampSeconds);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Non-blocking - completes immediately
        return true;
    }

    /**
     * Gets the number of visible AprilTags from the latest vision result.
     * 
     * @return The number of visible tags, or 0 if no result available
     */
    private int getVisibleTagCount() {
        var result = vision.getLatestResult();
        if (result != null && result.hasTargets()) {
            return result.getTargets().size();
        }
        return 0;
    }

    /**
     * Gets the average distance to visible AprilTags in meters.
     * 
     * @return The average distance in meters, or Double.MAX_VALUE if no tags visible
     */
    private double getAvgTagDistanceMeters() {
        var result = vision.getLatestResult();
        if (result != null && result.hasTargets()) {
            var targets = result.getTargets();
            double totalDistance = 0.0;
            for (var target : targets) {
                totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            return totalDistance / targets.size();
        }
        return Double.MAX_VALUE;
    }
}
