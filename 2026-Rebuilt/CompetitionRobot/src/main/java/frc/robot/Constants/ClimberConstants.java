package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for the Climber subsystem.
 * Contains predefined climber positions, motion limits, PID tuning parameters, and motion constraints.
 */
public class ClimberConstants {
    /**
     * Enumeration of predefined climber positions in rotations.
     * DOWN, ACQUIRE, and CLIMB positions for different climbing states.
     */
    public static enum ClimberPosition {
        DOWN(0),
        ACQUIRE(-50),
        CLIMB(-260);

        public final double value;

        private ClimberPosition(double value) {
            this.value = value;
        }
    }

    public static final double SCORING_MOVEMENT = -0.8;

    public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
    public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

    public static final double kP = 10; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO
    public static final double kS = 0.017964; // TODO
    public static final double kG = 0.321192; // TODO
    public static final double kV = 0.876084;// TODO
    public static final double kA = 0.206676;// TODO

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // ---- Talon FX: current limits, ramp, voltage ----
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 80;
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC = 0.2;
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;
}
