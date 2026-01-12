package frc.robot.Subsystems.Cameras;

// import static frc.robot.Constants.VisionConstants.APRILTAG_PIPELINE;
// import static frc.robot.Constants.VisionConstants.CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.CAMERA_POSITION;
import static frc.robot.Constants.VisionConstants.CAMERA_ROTATION;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * PhotonVision camera subsystem for AprilTag detection and robot pose estimation.
 * Replaces the previous Limelight implementation.
 */
public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    /**
     * Creates a new PhotonVision subsystem.
     * 
     * @param cameraName The name of the camera as configured in PhotonVision
     * @param pipelineIndex The pipeline index to use for AprilTag detection
     */
    public PhotonVision(String cameraName, int pipelineIndex) {
        camera = new PhotonCamera(cameraName);
        camera.setPipelineIndex(pipelineIndex);

        // Load the field layout for pose estimation
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

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
     * @return Whether or not PhotonVision has targets within its field of view
     */
    public boolean hasTargets() {
        var result = camera.getLatestResult();
        return result != null && result.hasTargets();
    }

    /**
     * @return Horizontal offset (yaw) from crosshair to target in degrees
     *         Note: This maps to Limelight's getX() method
     */
    public double getX() {
        var result = camera.getLatestResult();
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
        var result = camera.getLatestResult();
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
        var result = camera.getLatestResult();
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
        var result = camera.getLatestResult();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getBestCameraToTarget().getX();
        }
        return 0.0;
    }

    /**
     * @return Estimated y distance to target in meters
     */
    public double getYDistance() {
        var result = camera.getLatestResult();
        if (result != null && result.hasTargets()) {
            return result.getBestTarget().getBestCameraToTarget().getY();
        }
        return 0.0;
    }

    /**
     * @return Yaw (rotation relative to y-axis) of camera target in radians
     */
    public double getYaw() {
        var result = camera.getLatestResult();
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
        var result = camera.getLatestResult();
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
     * 
     * @return The latest PhotonPipelineResult
     */
    public org.photonvision.targeting.PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    /**
     * Gets the estimated robot pose from PhotonVision.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        var result = camera.getLatestResult();
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

    @Override
    public void periodic() {
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
