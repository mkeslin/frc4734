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

    // ---- Jaws (left and right, independent open/closed) ----
    /** Position tolerance in rotations for considering a jaw at goal. */
    public static final double JAW_POSITION_TOLERANCE_ROTATIONS = 0.1;

    /** Predefined jaw positions in rotations. Tune to match mechanism. */
    public static enum JawPosition {
        OPEN(0.0),
        CLOSED(1.0);

        public final double value;

        private JawPosition(double value) {
            this.value = value;
        }
    }

    /** MotionMagic cruise velocity for jaw motors (rot/s). */
    public static final double JAW_MOTION_MAGIC_CRUISE_VELOCITY = 20;
    /** MotionMagic acceleration for jaw motors (rot/s²). */
    public static final double JAW_MOTION_MAGIC_ACCELERATION = 40;
    /** MotionMagic jerk for jaw motors (rot/s³). */
    public static final double JAW_MOTION_MAGIC_JERK = 400;

    /** Jaw Slot0 kP for position control. */
    public static final double JAW_KP = 4.0;
    /** Jaw Slot0 kI. */
    public static final double JAW_KI = 0.0;
    /** Jaw Slot0 kD. */
    public static final double JAW_KD = 0.05;
    /** Jaw supply current limit (amps). */
    public static final double JAW_SUPPLY_CURRENT_LIMIT_AMPS = 25;
    /** Jaw stator current limit (amps). */
    public static final double JAW_STATOR_CURRENT_LIMIT_AMPS = 40;

    // ---- Climb cycle (rotation-based, run-while-held) ----
    /** Position tolerance in rotations for "at target" during climb/descend. */
    public static final double CLIMB_POSITION_TOLERANCE_ROTATIONS = 0.5;

    /** Level 1: extend target (head at bar), start target (retracted). */
    public static final double CLIMB_L1_EXTEND_ROTATIONS = ClimberPosition.ACQUIRE.value;
    public static final double CLIMB_L1_START_ROTATIONS = ClimberPosition.DOWN.value;
    /** Level 2: extend and start (tune to match mechanism). */
    public static final double CLIMB_L2_EXTEND_ROTATIONS = -150;
    public static final double CLIMB_L2_START_ROTATIONS = ClimberPosition.DOWN.value;
    /** Level 3: full extend and start. */
    public static final double CLIMB_L3_EXTEND_ROTATIONS = ClimberPosition.CLIMB.value;
    public static final double CLIMB_L3_START_ROTATIONS = ClimberPosition.DOWN.value;

    /** Descend v1: single target (retract to rest). Multi-level descend can use same L1/L2/L3 extend values in reverse. */
    public static final double DESCEND_TARGET_ROTATIONS = ClimberPosition.DOWN.value;
}
