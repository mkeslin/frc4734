package frc.robot.Constants;

/**
 * Constants for the Floor subsystem.
 * Contains predefined conveyor speeds for different operational modes.
 */
public class FloorConstants {
    /**
     * Enumeration of predefined conveyor speeds.
     * STOPPED, FORWARD (moves balls toward feeder/shooter), and REVERSE (moves balls away, for clearing jams).
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

    // Placeholder constants for future safety coordination
    // TODO: Add safety constants when safety conditions are determined
    // Examples:
    // public static final boolean REQUIRE_SHOOTER_READY = true;
    // public static final boolean STOP_WHEN_HOPPER_FULL = true;
}
