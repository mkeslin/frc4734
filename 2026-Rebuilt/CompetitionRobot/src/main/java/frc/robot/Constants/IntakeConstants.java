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
        DEPLOYED(1.0); // TODO: Update with actual deployed position value

        public final double value;

        private DeployPosition(double value) {
            this.value = value;
        }
    }

    /**
     * Enumeration of predefined intake speeds.
     * STOPPED, IN (intake), and OUT (outtake) speeds.
     */
    public static enum IntakeSpeed {
        STOPPED(0),
        IN(0.3),
        OUT(-0.3);

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
}
