package frc.robot.Constants;

/**
 * Constants for the DeployableIntake subsystem.
 * Contains predefined deploy positions and intake speeds for different operational modes.
 */
public class IntakeConstants {
    /**
     * Enumeration of predefined deploy positions in rotations.
     * These positions correspond to stowed and deployed states.
     */
    public static enum DeployPosition {
        STOWED(0),
        DEPLOYED(12);

        public final double value;

        private DeployPosition(double value) {
            this.value = value;
        }
    }

    /**
     * Enumeration of predefined intake speeds (rotations per second).
     * Values high enough to overcome static friction; tune for match as needed.
     */
    public static enum IntakeSpeed {
        STOPPED(0),
        IN(75.0),    // Intake; tune as needed
        OUT(-40.0);  // Outtake / reverse

        public final double value;

        private IntakeSpeed(double value) {
            this.value = value;
        }
    }

    // Deploy motor constants (similar to ArmConstants)
    public static final double DEPLOY_TOLERANCE = 0.1; // Tolerance for checking if deployed (rotations)
    
    // PID/MotionMagic constants for deploy motor (TODO: Tune these values)
    public static final double kP = 3.8;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;
    public static final double kG = -0.2; // Gravity compensation

    // Motion Magic settings for deploy motor
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 30; // rotations per second
    public static final double MOTION_MAGIC_ACCELERATION = 60; // rotations per second squared
    public static final double MOTION_MAGIC_JERK = 1600; // rotations per second cubed

    // Safety: minimum deploy position to allow intake operation (rotations)
    public static final double MIN_DEPLOY_POSITION_FOR_INTAKE = 0.5;

    /** When intake roller is running, deploy motor gets this forward voltage to keep pressure and hold intake in place (volts). */
    public static final double INTAKE_RUNNING_DEPLOY_HOLD_VOLTAGE = 0.5;

    // ---- Current limits, ramp, voltage (deploy and intake motors) ----
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 60;
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC = 0.2;
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;

    // ---- Intake roller: velocity Slot0 (for intake motor only) ----
    public static final double INTAKE_VELOCITY_KV = 0.12;
    public static final double INTAKE_VELOCITY_KS = 0.25;
    public static final double INTAKE_VELOCITY_KP = 0.1;
    public static final double INTAKE_VELOCITY_KI = 0.0;
    public static final double INTAKE_VELOCITY_KD = 0.0;
    public static final boolean INTAKE_MOTOR_INVERTED = true;
}
