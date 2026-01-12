package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Constants for the Arm subsystem.
 * Contains predefined arm positions, motion limits, and PID tuning parameters.
 */
public class ArmConstants {
    /**
     * Enumeration of predefined arm positions in rotations.
     * These positions correspond to different scoring levels and operational states.
     */
    public static enum ArmPosition {
        BOTTOM(0),
        L1(6.424805),
        L2(10.352051), // reef angle
        L3(10.352051),
        L3_SCORE(7.5),
        L4(11.641113),
        L4_SCORE(9.641113),
        TOP(13.1);

        public final double value;

        private ArmPosition(double value) {
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
}
