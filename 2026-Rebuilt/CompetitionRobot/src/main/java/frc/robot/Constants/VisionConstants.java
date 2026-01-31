package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Vision-related constants including camera pipelines and settings.
 *
 * <p><b>PhotonVision "Could not find any PhotonVision coprocessors"</b> means the robot cannot see
 * a PhotonVision instance on NetworkTables. Fix by:
 * <ul>
 *   <li>Running PhotonVision on a coprocessor (e.g. Raspberry Pi) with the camera connected</li>
 *   <li>Ensuring the camera name in the PhotonVision web UI exactly matches {@link #CAMERA_NAME}
 *       (currently "Webcam1")</li>
 *   <li>Connecting the coprocessor and RoboRIO to the same network (same team WiFi or USB)</li>
 *   <li>Checking PhotonVision docs: https://docs.photonvision.org/en/latest/docs/troubleshooting/networking-troubleshooting.html</li>
 * </ul>
 * <p><b>"Has not reported a message interface uuid - is your coprocessor's camera started?"</b> means
 * the coprocessor is visible but the camera for this name is not started. On the coprocessor:
 * open the PhotonVision web UI, add/select the camera named {@link #CAMERA_NAME}, and start the
 * camera (or enable "Start camera on boot" and restart PhotonVision).
 * <p>The robot code will run without vision (odometry only) when no coprocessor is found; check
 * SmartDashboard "Vision/Connected" to see if vision is available.
 */
public final class VisionConstants {
    private VisionConstants() {
        // Utility class - prevent instantiation
    }

    /** Pipeline index for AprilTag detection (0 = first pipeline, e.g. "Pipeline1" in PhotonVision UI) */
    public static final int APRILTAG_PIPELINE = 0;

    /** Camera name as configured in PhotonVision (must match the name in the PhotonVision web UI exactly) */
    public static final String CAMERA_NAME = "Webcam1";
    
    /** Camera position relative to robot center (in meters) */
    public static final Translation3d CAMERA_POSITION = new Translation3d(
        0.0,  // X: forward/backward offset
        0.0,  // Y: left/right offset
        0.0   // Z: up/down offset
    );
    
    /** Camera rotation relative to robot (in radians) */
    public static final Rotation3d CAMERA_ROTATION = new Rotation3d(
        0.0,  // Roll
        0.0,  // Pitch
        0.0   // Yaw
    );
    
    // Vision measurement filtering constants
    /** Maximum distance (in meters) a vision pose can be from current odometry pose before being rejected */
    public static final double MAX_POSE_JUMP_DISTANCE = 1.0; // meters
    
    /** Minimum number of tags detected for high-confidence vision measurements */
    public static final int MIN_TAGS_FOR_HIGH_CONFIDENCE = 2;
    
    /** Maximum acceptable age of vision measurement timestamp (in seconds) before being rejected */
    public static final double MAX_TIMESTAMP_AGE = 0.5; // seconds
}
