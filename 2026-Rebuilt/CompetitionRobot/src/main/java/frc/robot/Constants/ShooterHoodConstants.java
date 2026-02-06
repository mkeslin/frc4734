package frc.robot.Constants;

/**
 * Constants for the ShooterHood subsystem.
 * Uses an Actuonix L16-R 140mm 150:1 6V linear servo on roboRIO PWM (standard 1–2 ms servo pulse).
 * Position is set via servo PWM; the actuator holds position internally (no feedback to roboRIO).
 */
public final class ShooterHoodConstants {
    private ShooterHoodConstants() {
        // Utility class - prevent instantiation
    }

    /** PWM channel on the roboRIO (0–9 on main, 10+ on MXP). Use a PWM port, not PWM for motors. */
    public static final int PWM_CHANNEL = 0;

    /**
     * Predefined hood positions as servo setpoint (0.0 = retracted, 1.0 = extended).
     * Tune if your L16-R direction or mechanical limits differ.
     */
    public enum HoodPosition {
        /** Hood retracted (in). */
        IN(0.0),
        /** Hood extended (out). */
        OUT(1.0);

        /** Servo position in [0, 1] for WPILib Servo.setPosition(). */
        public final double value;

        HoodPosition(double value) {
            this.value = value;
        }
    }
}
