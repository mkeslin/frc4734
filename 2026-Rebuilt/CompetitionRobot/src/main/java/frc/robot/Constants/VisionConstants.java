package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Vision-related constants including camera pipelines and settings.
 */
public final class VisionConstants {
    private VisionConstants() {
        // Utility class - prevent instantiation
    }

    public static final int APRILTAG_PIPELINE = 0;
    
    /** Camera name as configured in PhotonVision */
    public static final String CAMERA_NAME = "photonvision";
    
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
