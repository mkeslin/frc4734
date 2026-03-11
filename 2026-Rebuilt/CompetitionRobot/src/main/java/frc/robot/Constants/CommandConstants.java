package frc.robot.Constants;

/**
 * Constants for command creation and binding configuration.
 * Contains timeouts, speeds, and timing values used throughout command sequences.
 */
public class CommandConstants {
    private CommandConstants() {
        // Utility class - prevent instantiation
    }

    // Command timing constants (in seconds)
    /** Default wait time for command synchronization (0.0 seconds - immediate) */
    public static final double DEFAULT_WAIT_TIME = 0.0;
    
    /** Timeout for short drivetrain movements during scoring */
    public static final double SHORT_DRIVE_TIMEOUT = 0.12;
    
    /** Timeout for medium drivetrain movements during scoring */
    public static final double MEDIUM_DRIVE_TIMEOUT = 0.15;
    
    /** Timeout for long drivetrain movements during scoring */
    public static final double LONG_DRIVE_TIMEOUT = 0.17;
    
    /** Timeout for post-score backward movement */
    public static final double POST_SCORE_BACKWARD_TIMEOUT = 0.35;
    
    /** Delay before moving arm to top position after intake */
    public static final double POST_INTAKE_ARM_DELAY = 0.40;

    /** When true, teleop shooter speed is computed from distance to hub (ShotModel). When false, uses fixed ShooterSpeed.FORWARD. Auto always uses per-motor values from Shuffleboard. */
    public static final boolean USE_DYNAMIC_SHOOTER_SPEED = true;
    /** TEMPORARY: When true, override shooter speed to 12 ft value (128.67 RPS). Set false to restore dynamic/fixed logic. */
    public static final boolean USE_TEMPORARY_6FT_SHOOTER_SPEED = false;
    /** TEMPORARY: Shooter RPS (60% of 115 for low-ceiling testing). */
    public static final double TEMPORARY_SHOOTER_SPEED_6FT_RPS = 185.0;

    /** Teleop shoot: feeder runs reverse this long first to back ball off shooter wheels (seconds). */
    public static final double SHOOT_FEEDER_BACKOFF = 0.5;
    /** Teleop shoot: time after shooter start before feeder runs (seconds). Longer delay improves first-shot accuracy for heavier flywheel. */
    public static final double SHOOT_FEEDER_DELAY = 1.7;
    /** Teleop shoot: time after shooter start before floor runs (seconds). Matches auto for consistent spin-up. */
    public static final double SHOOT_FLOOR_DELAY = 0.75;
    /** Pulsed feed: how long feeder runs per pulse (seconds). Long enough to push one ball. Set high for continuous. */
    public static final double SHOOT_FEEDER_PULSE_ON_SEC = 0.25;
    /** Pulsed feed: delay between pulses (seconds). Pause between shots for flywheel recovery. */
    public static final double SHOOT_FEEDER_PULSE_DELAY_SEC = 0.2;

    // Drivetrain speed constants (normalized -1.0 to 1.0)
    /** Speed for approaching scoring position */
    public static final double APPROACH_SCORE_SPEED = -0.5;
    
    /** Speed for placing coral forward movement */
    public static final double PLACE_CORAL_FORWARD_SPEED = 0.75;
    
    /** Speed for backing away after scoring */
    public static final double POST_SCORE_BACKWARD_SPEED = -1.0;
    
    /** Speed for stopping drivetrain */
    public static final double STOP_SPEED = 0.0;

    // Climber voltage constants (in volts)
    /** Voltage for slow climber movement (POV right) */
    public static final double CLIMBER_SLOW_VOLTAGE = -1.75;
    
    /** Voltage for fast climber movement (POV left) */
    public static final double CLIMBER_FAST_VOLTAGE = -3.5;
    
    /** Voltage for stopping climber */
    public static final double CLIMBER_STOP_VOLTAGE = 0.0;
}
