package frc.robot.Subsystems.Cameras;

// import static frc.robot.Constants.VisionConstants.APRILTAG_PIPELINE;
// import static frc.robot.Constants.VisionConstants.CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.CAMERA_POSITION;
import static frc.robot.Constants.VisionConstants.CAMERA_ROTATION;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PathPlanner.AllianceUtils;

/**
 * PhotonVision camera subsystem for AprilTag detection and robot pose estimation.
 * Replaces the previous Limelight implementation.
 */
public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout fieldLayout;
    private Alliance lastKnownAlliance;

    /**
     * Creates a new PhotonVision subsystem.
     * 
     * @param cameraName The name of the camera as configured in PhotonVision
     * @param pipelineIndex The pipeline index to use for AprilTag detection
     */
    public PhotonVision(String cameraName, int pipelineIndex) {
        camera = new PhotonCamera(cameraName);
        camera.setPipelineIndex(pipelineIndex);

        // Load the field layout for pose estimation using AllianceUtils
        // This will use the 2026 field layout (or kDefaultField as placeholder)
        fieldLayout = AllianceUtils.getFieldLayout();
        lastKnownAlliance = AllianceUtils.getAlliance();

        // Create camera-to-robot transform
        Transform3d robotToCamera = new Transform3d(CAMERA_POSITION, CAMERA_ROTATION);

        // Create pose estimator with multi-tag strategy for best accuracy
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
    }

    /**
     * Gets the latest camera result using the non-deprecated API.
     * Uses getAllUnreadResults() and returns the most recent result, or null if none available.
     * 
     * @return The latest PhotonPipelineResult, or null if no results are available
     */
    private PhotonPipelineResult getLatestResultInternal() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results != null && !results.isEmpty()) {
            // Return the last (most recent) result
            return results.get(results.size() - 1);
        }
        return null;
    }

    /**
     * @return Whether or not PhotonVision has targets within its field of view
     */
    public boolean hasTargets() {
        var result = getLatestResultInternal();
        return result != null && result.hasTargets();
    }

    /**
     * @return Horizontal offset (yaw) from crosshair to target in degrees
     *         Note: This maps to Limelight's getX() method
     */
    public double getX() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    /**
     * @return Vertical offset (pitch) from crosshair to target in degrees
     *         Note: This maps to Limelight's getY() method
     */
    public double getY() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getPitch();
        }
        return 0.0;
    }

    /**
     * @return Area value of the target as a percentage (0-100)
     *         Note: PhotonVision uses different metrics than Limelight
     */
    public double getArea() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            // PhotonVision provides area directly as a percentage
            return target.getArea();
        }
        return 0.0;
    }

    /**
     * @return Estimated x distance to target in meters
     */
    public double getXDistance() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getBestCameraToTarget().getX();
        }
        return 0.0;
    }

    /**
     * @return Estimated y distance to target in meters
     */
    public double getYDistance() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getBestCameraToTarget().getY();
        }
        return 0.0;
    }

    /**
     * @return Yaw (rotation relative to y-axis) of camera target in radians
     */
    public double getYaw() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return Math.toRadians(result.getBestTarget().getYaw());
        }
        return 0.0;
    }

    /**
     * Sets the LED mode (on/off).
     * Note: PhotonVision LED control is typically handled through the web interface.
     * 
     * @param on true to turn LED on, false to turn off
     */
    public void status(boolean on) {
        // PhotonVision LED control is typically done through the web interface
        // This method is kept for compatibility with existing code
    }

    /**
     * @return Current pipeline index
     */
    public double getPipeline() {
        return camera.getPipelineIndex();
    }

    /**
     * @return AprilTag ID of the best target, or -1 if no target
     */
    public double getAprilTagID() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getFiducialId();
        }
        return -1;
    }

    /**
     * Sets the pipeline index.
     * 
     * @param pipelineIndex The pipeline index to use
     */
    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
    }

    /**
     * Gets the latest camera result for direct access.
     * Uses the non-deprecated API internally.
     * 
     * @return The latest PhotonPipelineResult, or null if no results are available
     */
    public PhotonPipelineResult getLatestResult() {
        return getLatestResultInternal();
    }

    /**
     * Gets the estimated robot pose from PhotonVision.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        var result = getLatestResultInternal();
        if (result != null && result.hasTargets()) {
            return poseEstimator.update(result);
        }
        return Optional.empty();
    }

    /**
     * Gets the PhotonPoseEstimator for use in pose estimation.
     * 
     * @return The PhotonPoseEstimator instance
     */
    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Updates the field layout for pose estimation (e.g., when alliance changes).
     * 
     * @param layout The new AprilTag field layout
     */
    public void setFieldLayout(AprilTagFieldLayout layout) {
        poseEstimator.setFieldTags(layout);
    }

    /**
     * Publishes diagnostic information to SmartDashboard.
     */
    public void putNums() {
        SmartDashboard.putNumber("photon-x", getX());
        SmartDashboard.putNumber("photon-y", getY());
        SmartDashboard.putNumber("photon-area", getArea());
        SmartDashboard.putNumber("photon-tag-id", getAprilTagID());
    }

    /**
     * Checks if the alliance has changed and updates the field layout if needed.
     * For rotationally symmetrical fields, the same layout is used for both alliances,
     * but this method ensures the layout is current and handles any future changes.
     */
    private void checkAndUpdateAlliance() {
        Alliance currentAlliance = AllianceUtils.getAlliance();
        if (currentAlliance != lastKnownAlliance) {
            // Alliance changed - update field layout
            fieldLayout = AllianceUtils.getFieldLayout();
            poseEstimator.setFieldTags(fieldLayout);
            lastKnownAlliance = currentAlliance;
        }
    }

    @Override
    public void periodic() {
        // Check for alliance changes and update field layout if needed
        checkAndUpdateAlliance();
        
        // Update diagnostics periodically
        putNums();
    }

    // Placeholder methods for compatibility (not implemented)
    public boolean canShootAmp() {
        return false;
    }

    public boolean canShootSpeaker() {
        return false;
    }

    public boolean canClimb() {
        return false;
    }

    /**
     * Cleans up resources when the robot is disabled.
     * PhotonCamera handles its own cleanup, but this method is provided
     * for consistency with other subsystems.
     */
    public void cleanup() {
        // PhotonCamera automatically handles cleanup when robot shuts down
        // No explicit cleanup needed, but method provided for consistency
    }
}
