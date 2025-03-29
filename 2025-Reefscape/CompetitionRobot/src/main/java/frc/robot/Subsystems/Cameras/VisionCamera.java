package frc.robot.Subsystems.Cameras;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionCamera extends SubsystemBase {
    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_poseEstimator;
    private Matrix<N3, N1> m_curStdDevs;

    private List<PhotonPipelineResult> m_cameraResults;

    // Simulation
    private PhotonCameraSim m_cameraSim;
    private VisionSystemSim m_visionSim;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public VisionCamera(String cameraName, int pipelineIndex) {
        m_camera = new PhotonCamera(cameraName);
        m_camera.setPipelineIndex(pipelineIndex);

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        var robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // PhotonPoseEstimator
        // note: change this for each year
        var aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        m_poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            m_visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            m_visionSim.addAprilTags(aprilTagFieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Update the linked PhotonCamera's values with visible targets.
            m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            m_visionSim.addCamera(m_cameraSim, robotToCam);

            m_cameraSim.enableDrawWireframe(true);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : m_cameraResults) {
            visionEst = m_poseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }

        return visionEst;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            m_curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = m_poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                m_curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                m_curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return m_curStdDevs;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ----- Simulation
    public void simulationPeriodic(Pose2d robotSimPose) {
        m_visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            m_visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return m_visionSim.getDebugField();
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public void cameraPeriodic() {
        m_cameraResults = m_camera.getAllUnreadResults();
    }

    private PhotonPipelineResult getLatestResult() {
        return m_cameraResults.get(m_cameraResults.size() - 1);
    }

    private PhotonTrackedTarget getBestTarget() {
        var cameraResult = getLatestResult();
        return cameraResult.getBestTarget();
    }

    /**
     * @return Whether or not camera has targets within its field of view
     */
    public boolean hasTargets() {
        var result = getLatestResult();
        return result.hasTargets();
    }

    /**
     * @return X value of camera
     */
    public double getX() {
        var cameraTarget = getBestTarget();
        var cameraTransform = cameraTarget.getBestCameraToTarget();
        return cameraTransform.getX();
    }

    /**
     * @return Y value of camera
     */
    public double getY() {
        var cameraTarget = getBestTarget();
        var cameraTransform = cameraTarget.getBestCameraToTarget();
        return cameraTransform.getY();
    }

    /*
     * public static double getTargetYaw() {
     * return(table.getEntry("camerapose_targetspace").getDoubleArray(new double[6])[5]);
     * }
     */

    /**
     * @return Area value of camera
     */
    public double getArea() {
        var cameraTarget = getBestTarget();
        return cameraTarget.getArea();
    }

    // /**
    // * @return Estimated x distance to target in meters
    // */
    // public double getXDistance() {
    // return LimelightHelpers.getTargetPose3d_CameraSpace(tableName).getX();
    // }

    // /**
    // * @return Estimated y distance to target in meters
    // */
    // public double getYDistance() {
    // return LimelightHelpers.getTargetPose3d_CameraSpace(tableName).getY();
    // }

    /**
     * @return yaw (rotation relative to y-axis) of camera camera target in radians
     */
    public double getYaw() {
        var cameraTarget = getBestTarget();
        return cameraTarget.getYaw();
    }

    /**
     * Turns the camera on and off
     * 
     * @param b
     *            true if on, false if off
     */
    // public void status(boolean b) {
    // if (b) {
    // LimelightHelpers.setLEDMode_ForceOn(tableName);
    // } else {
    // LimelightHelpers.setLEDMode_ForceOff(tableName);
    // }
    // }

    // public double getPipeline() {
    // return LimelightHelpers.getCurrentPipelineIndex(tableName);
    // }

    public double getAprilTagID() {
        var cameraTarget = getBestTarget();
        return cameraTarget.getFiducialId();
    }

    // public void setPipeline(int p) {
    // LimelightHelpers.setPipelineIndex(tableName, p);
    // }

    // public Pose2d getRobotMegaTagPose() {
    // return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tableName).pose;
    // }

    public void putNums() {
        SmartDashboard.putNumber("lime-x", getX());
        SmartDashboard.putNumber("lime-y", getY());
        // SmartDashboard.putNumber("lime-target-yaw", getTargetYaw());
        SmartDashboard.putNumber("lime-area", getArea());
    }
}
