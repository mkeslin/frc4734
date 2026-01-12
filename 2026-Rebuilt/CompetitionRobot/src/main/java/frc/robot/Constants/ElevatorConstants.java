package frc.robot.Constants;

public class ElevatorConstants {
    public static enum ElevatorPosition {
        BOTTOM(0),
        INTAKE_PREP(4.283203 ),
        INTAKE(0.355),
        INTAKE_POST(20.0),
        L1(0.355),
        L2(0.355),
        L3(7.0),
        L4(20.5),
        TOP(22.021484);

        public final double value;

        private ElevatorPosition(double value) {
            this.value = value;
        }
    }

    public static final double SCORING_MOVEMENT = -0.25;

    public static final double MIN_HEIGHT_METERS = 0.005; // TODO
    public static final double MAX_HEIGHT_METERS = 1.57; // TODO

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
}