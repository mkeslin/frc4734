package frc.robot.Constants;

/**
 * Constants for the Shooter subsystem.
 * Contains predefined shooter speeds and velocity closed-loop gains for Slot0.
 */
public class ShooterConstants {
    private ShooterConstants() {}

    /**
     * Enumeration of predefined shooter speeds (rotations per second).
     * STOPPED, FORWARD (shoots balls forward), and REVERSE (reverses for clearing jams).
     * Values must be high enough that kV * speed + kS overcomes static friction (~1–2 V minimum).
     */
    public static enum ShooterSpeed {
        STOPPED(0),
        /** RPS for shooting; tune for desired shot distance. */
        FORWARD(7),//24.0),
        /** RPS for reverse (e.g. unjam); tune as needed. */
        REVERSE(-80.0);

        public final double value;

        private ShooterSpeed(double value) {
            this.value = value;
        }
    }

    // ---- Velocity closed-loop (Slot0) ----
    // Tune via SysId quasistatic/dynamic or manual tuning. These are safe placeholders.
    /** Velocity feedforward: volts per (rps). Approx 12V / free-speed-rps. */
    public static final double VELOCITY_KV = 0.12;
    /** Static friction feedforward (volts). Minimum voltage to overcome friction; helps at low speed. */
    public static final double VELOCITY_KS = 0.25;
    /** Proportional gain on velocity error. */
    public static final double VELOCITY_KP = 0.1;
    /** Integral gain (use 0 or small to avoid windup). */
    public static final double VELOCITY_KI = 0.0;
    /** Derivative gain on velocity error rate. */
    public static final double VELOCITY_KD = 0.0;

    // ---- Current limits ----
    // Protects battery/breaker and reduces brownout risk. Tune to your breaker and load.
    /** Supply current limit (amps). Current drawn from the bus. */
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30;
    /** Enable supply current limiting. */
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    /** Stator current limit (amps). Motor current; limits torque (e.g. during jam). */
    public static final double STATOR_CURRENT_LIMIT_AMPS = 50;
    /** Enable stator current limiting. */
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;

    // ---- Motor output ----
    /** Set true to invert motor direction (use if positive velocity spins wrong way). */
    public static final boolean MOTOR_INVERTED = true;

    // ---- Closed-loop ramp ----
    /** Time (seconds) for closed-loop voltage output to ramp 0→12V. 0 = no ramp (instant). Smooths spin-up and can reduce current spikes. */
    public static final double CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC = 0.2;

    // ---- Peak output voltage ----
    /** Peak forward voltage (volts). Slightly below 12 to leave bus headroom and reduce brownout risk. */
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    /** Peak reverse voltage (volts), typically negative. */
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;
}
