package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Alliance-aware landmark positions for the robot.
 * 
 * <p>All landmarks are defined in blue alliance coordinates and automatically
 * transformed to the current alliance's coordinate system using {@link AllianceUtils}.
 * 
 * <p>For the 2026 rotationally symmetrical field, poses are rotated 180 degrees
 * around the field center when on the red alliance.
 */
public class Landmarks {

    /**
     * Gets the starting position 1 for the current alliance.
     * 
     * @return The starting position 1 in the current alliance's coordinate system
     */
    public static Pose2d OurStart1() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Start1);
    }

    /**
     * Gets the starting position 2 for the current alliance.
     * 
     * @return The starting position 2 in the current alliance's coordinate system
     */
    public static Pose2d OurStart2() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Start2);
    }

    /**
     * Gets the starting position 3 for the current alliance.
     * 
     * @return The starting position 3 in the current alliance's coordinate system
     */
    public static Pose2d OurStart3() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Start3);
    }

    /**
     * Gets the hub (scoring target / diamond plate) center for the current alliance.
     */
    public static Pose2d OurHub() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Hub);
    }

    /**
     * Gets the tower (uprights reference) pose for the current alliance.
     */
    public static Pose2d OurTower() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Tower);
    }

    /**
     * Gets the field center for the current alliance.
     */
    public static Pose2d OurFieldCenter() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.FieldCenter);
    }

    /**
     * Gets the upright near hub, top, for the current alliance.
     */
    public static Pose2d OurUprightNearHubTop() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.UprightNearHubTop);
    }

    /**
     * Gets the upright near hub, bottom, for the current alliance.
     */
    public static Pose2d OurUprightNearHubBottom() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.UprightNearHubBottom);
    }

    /**
     * Gets our feeder note stack bottom center for the current alliance.
     */
    public static Pose2d OurFeederStackBottom() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.FeederStackBottom);
    }

    /**
     * Gets our feeder note stack top center for the current alliance.
     */
    public static Pose2d OurFeederStackTop() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.FeederStackTop);
    }

    /**
     * Gets our alliance station entry, bottom center, for the current alliance.
     */
    public static Pose2d OurEntryBottom() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.EntryBottom);
    }

    /**
     * Gets our alliance station entry, top center, for the current alliance.
     */
    public static Pose2d OurEntryTop() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.EntryTop);
    }

    /**
     * Gets the center note stack center for the current alliance.
     */
    public static Pose2d OurCenterStackCenter() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.CenterStackCenter);
    }
}
