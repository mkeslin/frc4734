package frc.robot.Constants;

/**
 * Constants for the Feeder subsystem.
 * Contains predefined feeder speeds for different operational modes.
 */
public class FeederConstants {
    /**
     * Enumeration of predefined feeder speeds.
     * STOPPED, FORWARD (moves balls toward shooter), and REVERSE (moves balls away, for clearing jams).
     */
    public static enum FeederSpeed {
        STOPPED(0),
        FORWARD(0.3),  // TODO: Tune speed value
        REVERSE(-0.3); // TODO: Tune speed value

        public final double value;

        private FeederSpeed(double value) {
            this.value = value;
        }
    }

    // Placeholder constants for future safety coordination
    // TODO: Add safety constants when safety conditions are determined
    // Examples:
    // public static final boolean REQUIRE_SHOOTER_READY = true;
    // public static final boolean COORDINATE_WITH_FLOOR = true;
}
