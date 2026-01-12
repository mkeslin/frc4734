package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimberConstants {
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
}
