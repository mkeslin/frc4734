package frc.robot.Constants;

/**
 * Constants for the Floor subsystem.
 * Contains predefined conveyor speeds and Talon FX config (Slot0, current limits, ramp, voltage).
 */
public class FloorConstants {
    private FloorConstants() {}

    /**
     * Enumeration of predefined conveyor speeds (rotations per second).
     */
    public static enum ConveyorSpeed {
        STOPPED(0),
        FORWARD(0.3),  // TODO: Tune speed value
        REVERSE(-0.3); // TODO: Tune speed value

        public final double value;

        private ConveyorSpeed(double value) {
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
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 60;
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;

    // ---- Motor output ----
    public static final boolean MOTOR_INVERTED = false;

    // ---- Closed-loop ramp ----
    public static final double CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC = 0.2;

    // ---- Peak output voltage ----
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;
}
