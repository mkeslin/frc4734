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

    // TODO: Add 2026 field landmark methods (hub, tower, etc.) as needed
}
