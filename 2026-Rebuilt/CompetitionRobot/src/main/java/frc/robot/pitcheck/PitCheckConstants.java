package frc.robot.pitcheck;

/**
 * Constants for pit check testing framework.
 * 
 * All durations are in seconds.
 * All power levels are in the range [-1.0, 1.0] for motors.
 * All thresholds are in appropriate units (amps, RPM, encoder counts, etc.).
 */
public final class PitCheckConstants {
    private PitCheckConstants() {
        // Utility class
    }

    // Overall timing
    /** Maximum total time for entire pit check (seconds) */
    public static final double MAX_TOTAL_TIME = 45.0;
    
    /** Time to wait between steps (seconds) */
    public static final double STEP_DELAY = 0.1;

    // Gate check
    /** Timeout for gate check step (seconds) */
    public static final double GATE_CHECK_TIMEOUT = 2.0;

    // Drive module tests
    /** Drive pulse duration (seconds) */
    public static final double DRIVE_PULSE_DURATION = 0.35;
    
    /** Drive pulse power (low power for safety) */
    public static final double DRIVE_PULSE_POWER = 0.15;
    
    /** Minimum encoder delta for drive pulse (counts or meters) */
    public static final double DRIVE_MIN_ENCODER_DELTA = 0.01; // meters or equivalent
    
    /** Minimum current draw for drive motor (amps) - detects unplugged */
    public static final double DRIVE_MIN_CURRENT = 0.5;
    
    /** Maximum current draw for drive motor (amps) - detects jam */
    public static final double DRIVE_MAX_CURRENT = 30.0;

    /** Steer pulse duration (seconds) */
    public static final double STEER_PULSE_DURATION = 0.35;
    
    /** Steer pulse angle change (radians) */
    public static final double STEER_PULSE_ANGLE = 0.1; // ~6 degrees
    
    /** Minimum steer encoder delta (radians) */
    public static final double STEER_MIN_ENCODER_DELTA = 0.05;

    // Intake test
    /** Intake pulse duration (seconds) */
    public static final double INTAKE_PULSE_DURATION = 0.5;
    
    /** Intake pulse power */
    public static final double INTAKE_PULSE_POWER = 0.3;
    
    /** Minimum current for intake (amps) */
    public static final double INTAKE_MIN_CURRENT = 0.3;
    
    /** Maximum current for intake (amps) */
    public static final double INTAKE_MAX_CURRENT = 15.0;

    // Feeder test
    /** Feeder pulse duration (seconds) */
    public static final double FEEDER_PULSE_DURATION = 0.5;
    
    /** Feeder pulse power */
    public static final double FEEDER_PULSE_POWER = 0.3;
    
    /** Minimum current for feeder (amps) */
    public static final double FEEDER_MIN_CURRENT = 0.3;
    
    /** Maximum current for feeder (amps) */
    public static final double FEEDER_MAX_CURRENT = 15.0;

    // Shooter test
    /** Shooter spin duration (seconds) */
    public static final double SHOOTER_SPIN_DURATION = 1.0;
    
    /** Shooter test RPM (modest, not full power) */
    public static final double SHOOTER_TEST_RPM = 2000.0;
    
    /** Minimum RPM threshold after spinup (RPM) */
    public static final double SHOOTER_MIN_RPM = 1500.0;
    
    /** Time to wait before checking RPM (seconds) */
    public static final double SHOOTER_RPM_CHECK_DELAY = 0.3;

    // Vision test
    /** Maximum time since last vision update to consider camera alive (seconds) */
    public static final double VISION_MAX_AGE = 2.0;
    
    /** Minimum tag count to consider vision working (optional) */
    public static final int VISION_MIN_TAG_COUNT = 0; // 0 = just check camera alive

    // Climber test
    /** Climber pulse duration if motion allowed (seconds) */
    public static final double CLIMBER_PULSE_DURATION = 0.3;
    
    /** Climber pulse power */
    public static final double CLIMBER_PULSE_POWER = 0.2;
    
    /** Minimum current for climber (amps) */
    public static final double CLIMBER_MIN_CURRENT = 0.3;
    
    /** Maximum current for climber (amps) */
    public static final double CLIMBER_MAX_CURRENT = 20.0;

    // Telemetry sampling
    /** Sample rate for telemetry during pulse tests (Hz) */
    public static final double TELEMETRY_SAMPLE_RATE = 50.0; // 50 Hz = 20ms intervals
    
    /** Number of samples to collect before evaluating */
    public static final int MIN_SAMPLES_FOR_EVALUATION = 5;
}
