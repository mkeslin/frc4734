package frc.robot.Constants;

public class SideToSideConstants {
    public static enum SideToSidePosition {
        LEFT(28.3),
        CENTER(0),
        RIGHT(-28.3);

        public final double value;

        private SideToSidePosition(double value) {
            this.value = value;
        }
    }

    public static final double ROTATIONS_PER_INCH = 4.697340;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
}