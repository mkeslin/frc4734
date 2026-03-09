package frc.robot.Auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constants for autonomous commands.
 * Contains default timeouts, tolerances, vision thresholds, and field element positions.
 * Commands in this package are used for autonomous routines and time-based sequences (not teleop bindings).
 */
public final class AutoConstants {
    private AutoConstants() {
        // Utility class - prevent instantiation
    }

    /** Path name (without .path) for drivetrain path-following PID tuning. File: zzTuning-1.path. */
    public static final String TUNING_PATH_NAME = "zzTuning-1";

    /** TEMPORARY: max velocity (m/s) for auto path following. Use lower value for testing; revert to 5.0 for competition. */
    public static final double AUTO_PATH_MAX_VELOCITY_MPS = 3.0;
    /** TEMPORARY: max acceleration (m/s²) for auto path following. Use lower value for testing; revert to 3.0 for competition. */
    public static final double AUTO_PATH_MAX_ACCELERATION_MPS2 = 2.0;

    // Default timeouts (seconds)
    public static final double DEFAULT_PATH_TIMEOUT = 10.0;
    public static final double DEFAULT_POSE_TIMEOUT = 5.0;
    public static final double DEFAULT_HEADING_TIMEOUT = 3.0;
    public static final double DEFAULT_SHOOTER_SPINUP_TIMEOUT = 3.0;
    public static final double DEFAULT_SHOOT_TIMEOUT = 2.0;
    public static final double DEFAULT_INTAKE_TIMEOUT = 5.0;
    /** Timeout for deploying (lowering) intake so webcam is unblocked before hub aim. */
    public static final double DEFAULT_INTAKE_DEPLOY_TIMEOUT = 2.0;
    /** Distance (m) to drive forward after tower align to acquire the bar before climbing. */
    public static final double DEFAULT_CLIMB_ACQUIRE_DISTANCE_METERS = 0.2;
    /** Offset (m): tower align pose is this far toward field center from the bar so robot extends climber there, then drives to bar. 2 ft. */
    public static final double CLIMB_TOWER_OFFSET_TOWARD_CENTER_METERS = 0.6096;
    /** Distance (m) to drive from offset pose toward the bar after extending climber (same as offset). 2 ft. */
    public static final double CLIMB_DRIVE_TO_BAR_METERS = 0.6096;
    /** Extra drive (m) toward the bar after initial drive, so robot has enough bar to climb on. ~6 in total. */
    public static final double CLIMB_EXTRA_DRIVE_TOWARD_BAR_METERS = 0.1524;
    /** Final nudge (m) toward the bar (-X) and +Y right before retract. 2 in each direction. */
    public static final double CLIMB_DRIVE_BEFORE_RETRACT_METERS = 0.0508;
    /** Timeout for the drive-forward-to-acquire-bar step. */
    public static final double DEFAULT_CLIMB_ACQUIRE_TIMEOUT = 3.0;
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

    /** TEMPORARY: shooter target speed (RPS) for low-ceiling room testing. Revert to 3000 and 100 tolerance for competition. */
    public static final double TEMPORARY_SHOOTER_TARGET_SPEED = 130.0;
    /** TEMPORARY: tolerance for at-speed check when using low-ceiling speed. */
    public static final double TEMPORARY_SHOOTER_TOLERANCE = 5.0;

    // Hub position (field-relative, blue alliance origin).
    public static final Pose2d HUB_POSE = new Pose2d(
            new Translation2d(4.620, 4.035),
            Rotation2d.fromDegrees(0)
    );
}
