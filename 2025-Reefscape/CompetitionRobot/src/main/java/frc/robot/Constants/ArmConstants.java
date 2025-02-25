package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static enum ArmPosition {
        // BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
        // HORIZONTAL(0),
        // L1(0),
        // L2(Units.degreesToRadians(55)), // reef angle
        // L3(Units.degreesToRadians(55)),
        // L4(1.033),
        // TOP(Math.PI / 2.0);

        BOTTOM(0),

        // PREINTAKE(.2),

        // HORIZONTAL(0),
        L1(6.424805),
        L2(10.352051), // reef angle

        L3(10.352051),
        L3_SCORE(7.5),

        L4(11.641113),
        L4_SCORE(9.641113),

        TOP(13.427246);

        public final double value;

        private ArmPosition(double value) {
            this.value = value;
        }
    }

    // public static final double MOTION_LIMIT = -0.7;
    public static final double SCORING_MOVEMENT = -0.8;

    // public static final int MOTOR_ID = 12;
    // public static final boolean MOTOR_INVERTED = true;

    // public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
    // public static final double GEARING = 40.0; // TODO
    // public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
    // public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
    // public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
    // public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

    public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
    public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

    // public static final int CURRENT_LIMIT = 50;

    public static final double kP = 10; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO
    public static final double kS = 0.017964; // TODO
    public static final double kG = 0.321192; // TODO
    public static final double kV = 0.876084;// TODO
    public static final double kA = 0.206676;// TODO
    // public static final double TOLERANCE = 0.02;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
}
