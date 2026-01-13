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
     * Gets the coral station 1 position for the current alliance.
     * 
     * @return The coral station 1 position in the current alliance's coordinate system
     */
    public static Pose2d CoralStation1() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.CoralStation1);
    }

    /**
     * Gets the coral station 2 position for the current alliance.
     * 
     * @return The coral station 2 position in the current alliance's coordinate system
     */
    public static Pose2d CoralStation2() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.CoralStation2);
    }

    /**
     * Gets the reef 1 position for the current alliance.
     * 
     * @return The reef 1 position in the current alliance's coordinate system
     */
    public static Pose2d Reef1() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Reef1);
    }

    /**
     * Gets the reef 2 position for the current alliance.
     * 
     * @return The reef 2 position in the current alliance's coordinate system
     */
    public static Pose2d Reef2() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Reef2);
    }

    /**
     * Gets the reef 3 position for the current alliance.
     * 
     * @return The reef 3 position in the current alliance's coordinate system
     */
    public static Pose2d Reef3() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Reef3);
    }

    /**
     * Gets the reef 4 position for the current alliance.
     * 
     * @return The reef 4 position in the current alliance's coordinate system
     */
    public static Pose2d Reef4() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Reef4);
    }

    /**
     * Gets the reef 5 position for the current alliance.
     * 
     * @return The reef 5 position in the current alliance's coordinate system
     */
    public static Pose2d Reef5() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Reef5);
    }

    /**
     * Gets the reef 6 position for the current alliance.
     * 
     * @return The reef 6 position in the current alliance's coordinate system
     */
    public static Pose2d Reef6() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Reef6);
    }
}
