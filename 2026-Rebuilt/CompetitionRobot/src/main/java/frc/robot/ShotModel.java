package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PathPlanner.Landmarks;

/**
 * Shot model that computes shooter speed (RPS) from distance to the hub.
 * Empirical fit: RPS = 3.5 * distanceFt + 85 (8'→113, 10'→120, 14'→134, 18'→148).
 */
public final class ShotModel {
    private ShotModel() {}

    /** Slope: RPS per foot of distance. */
    private static final double SLOPE = 3.5;
    /** Intercept: base RPS at zero distance. */
    private static final double INTERCEPT = 85.0;

    /** Min distance (ft) for formula; below this, use min RPS. */
    private static final double MIN_DISTANCE_FT = 3.0;
    /** Max distance (ft) for formula; above this, use max RPS. */
    private static final double MAX_DISTANCE_FT = 25.0;
    /** Fallback RPS when distance cannot be computed. */
    private static final double FALLBACK_RPS = 110.0;

    /**
     * Computes shooter RPS from distance in feet.
     *
     * @param distanceFeet Distance to hub in feet
     * @return Target shooter speed in RPS (rotations per second)
     */
    public static double rpsFromDistanceFeet(double distanceFeet) {
        if (distanceFeet < MIN_DISTANCE_FT) {
            return rpsFromDistanceFeet(MIN_DISTANCE_FT);
        }
        if (distanceFeet > MAX_DISTANCE_FT) {
            return rpsFromDistanceFeet(MAX_DISTANCE_FT);
        }
        return SLOPE * distanceFeet + INTERCEPT;
    }

    /**
     * Computes shooter RPS from distance in meters.
     *
     * @param distanceMeters Distance to hub in meters
     * @return Target shooter speed in RPS
     */
    public static double rpsFromDistanceMeters(double distanceMeters) {
        double distanceFeet = distanceMeters / 0.3048;
        return rpsFromDistanceFeet(distanceFeet);
    }

    /**
     * Computes shooter RPS from robot and hub translations (same coordinate frame).
     *
     * @param robotTranslation Robot position
     * @param hubTranslation   Hub position
     * @return Target shooter speed in RPS, or fallback if distance is invalid
     */
    public static double rpsFromPose(Translation2d robotTranslation, Translation2d hubTranslation) {
        double distanceMeters = robotTranslation.getDistance(hubTranslation);
        if (Double.isNaN(distanceMeters) || distanceMeters < 0) {
            return FALLBACK_RPS;
        }
        return rpsFromDistanceMeters(distanceMeters);
    }

    /**
     * Computes shooter RPS from robot pose using alliance-aware hub position.
     *
     * @param robotTranslation Robot position (must match hub coordinate frame)
     * @return Target shooter speed in RPS
     */
    public static double rpsFromRobotToHub(Translation2d robotTranslation) {
        Translation2d hubTranslation = Landmarks.OurHub().getTranslation();
        return rpsFromPose(robotTranslation, hubTranslation);
    }
}
