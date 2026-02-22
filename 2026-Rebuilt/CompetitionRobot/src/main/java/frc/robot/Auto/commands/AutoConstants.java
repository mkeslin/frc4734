package frc.robot.Auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constants for autonomous commands.
 * Contains default timeouts, tolerances, vision thresholds, and field element positions.
 */
public final class AutoConstants {
    private AutoConstants() {
        // Utility class - prevent instantiation
    }

    /** Path name (without .path) for drivetrain path-following PID tuning. File: zzTuning-1.path. */
    public static final String TUNING_PATH_NAME = "zzTuning-1";

    // Default timeouts (seconds)
    public static final double DEFAULT_PATH_TIMEOUT = 10.0;
    public static final double DEFAULT_POSE_TIMEOUT = 5.0;
    public static final double DEFAULT_HEADING_TIMEOUT = 3.0;
    public static final double DEFAULT_SHOOTER_SPINUP_TIMEOUT = 3.0;
    public static final double DEFAULT_SHOOT_TIMEOUT = 2.0;
    public static final double DEFAULT_INTAKE_TIMEOUT = 5.0;
    public static final double DEFAULT_CLIMB_TIMEOUT = 15.0;
    public static final double DEFAULT_ODOMETRY_SEED_TIMEOUT = 1.0;

    // Position tolerances (meters)
    public static final double DEFAULT_XY_TOLERANCE = 0.1; // 10 cm
    public static final double DEFAULT_ROTATION_TOLERANCE = Math.toRadians(5.0); // 5 degrees

    // Vision thresholds
    public static final double DEFAULT_MAX_AMBIGUITY = 0.2;
    public static final double DEFAULT_MAX_TAG_DISTANCE = 5.0; // meters
    public static final int DEFAULT_MIN_TARGETS = 2;

    // Heading control
    public static final int DEFAULT_STABLE_FRAMES = 10; // frames at target before considered stable
    public static final double DEFAULT_FALLBACK_HEADING_DEG = 180.0; // degrees

    // Hub position (field-relative, blue alliance origin). From 2026 field diagram: blue diamond plate center 197.61", 158.845" (m)
    public static final Pose2d HUB_POSE = new Pose2d(
            new Translation2d(5.019, 4.035),
            Rotation2d.fromDegrees(0)
    );
}
