package frc.robot.Constants;

/**
 * Constants for the Feeder subsystem.
 * Contains predefined feeder speeds and Talon FX config (Slot0, current limits, ramp, voltage).
 */
public class FeederConstants {
    private FeederConstants() {}

    /**
     * Enumeration of predefined feeder speeds (rotations per second).
     * Values high enough to overcome static friction; tune for match as needed.
     */
    public static enum FeederSpeed {
        STOPPED(0),
        FORWARD(200.0),   // Toward shooter; tune for indexing
        REVERSE(-60.0);  // Unjam / reverse

        public final double value;

        private FeederSpeed(double value) {
            this.value = value;
        }
    }

    // ---- Velocity closed-loop (Slot0) ----
    public static final double VELOCITY_KV = 0.12;
    public static final double VELOCITY_KS = 0.25;
    public static final double VELOCITY_KP = 0.1;
    public static final double VELOCITY_KI = 0.0;
    public static final double VELOCITY_KD = 0.0;

    // ---- Current limits ----
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 35;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 50;
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;

    // ---- Motor output ----
    public static final boolean MOTOR_INVERTED = false;

    // ---- Closed-loop ramp ----
    public static final double CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC = 0.2;

    // ---- Peak output voltage ----
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;
}
