package frc.robot.Constants;

/**
 * Constants for the Shooter subsystem.
 * Contains predefined shooter speeds for different operational modes.
 */
public class ShooterConstants {
    /**
     * Enumeration of predefined shooter speeds.
     * STOPPED, FORWARD (shoots balls forward), and REVERSE (reverses for clearing jams).
     */
    public static enum ShooterSpeed {
        STOPPED(0),
        FORWARD(0.5),  // TODO: Tune speed value for shooting
        REVERSE(-0.3); // TODO: Tune speed value for reverse

        public final double value;

        private ShooterSpeed(double value) {
            this.value = value;
        }
    }

    // Placeholder constants for future safety coordination
    // TODO: Add safety constants when safety conditions are determined
    // Examples:
    // public static final boolean REQUIRE_FEEDER_READY = true;
    // public static final boolean CHECK_BALL_PRESENCE = true;
}
