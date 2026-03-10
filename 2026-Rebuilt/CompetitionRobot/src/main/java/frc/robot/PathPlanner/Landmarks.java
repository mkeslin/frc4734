package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Auto.commands.ClimbSide;
import frc.robot.Auto.commands.StartPoseId;

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
     * Gets the shot position (in front of hub, facing hub) for the current alliance.
     */
    public static Pose2d OurShotPosition() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.ShotPosition);
    }

    /** Gets the left shooter auto shot position for the current alliance. */
    public static Pose2d OurShotPositionLeft() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.ShotPositionLeft);
    }

    /** Gets the center shooter auto shot position for the current alliance. */
    public static Pose2d OurShotPositionCenter() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.ShotPositionCenter);
    }

    /** Gets the right shooter auto shot position for the current alliance. */
    public static Pose2d OurShotPositionRight() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.ShotPositionRight);
    }

    /**
     * Shot pose equidistant between start and tower align: midpoint of the two positions,
     * with rotation facing the hub so the robot can aim and shoot.
     * Used by ClimberAuto so shot position depends on start and selected climb side.
     *
     * @param startPose Start pose (e.g. OurStart1() for this routine)
     * @param towerAlignPose Tower align pose for selected climb side (OurTowerAlignLeft() or OurTowerAlignRight())
     * @return Pose at midpoint, facing the hub
     */
    public static Pose2d midpointShotPose(Pose2d startPose, Pose2d towerAlignPose) {
        Translation2d mid = startPose.getTranslation().plus(towerAlignPose.getTranslation()).div(2);
        Translation2d hub = OurHub().getTranslation();
        Rotation2d towardHub = hub.minus(mid).getAngle();
        return new Pose2d(mid, towardHub);
    }

    /**
     * Shot pose in blue coordinates only: midpoint between blue start and blue tower align,
     * rotation toward blue hub. Use this when the target is passed to PathPlanner pathfinding,
     * which expects blue-origin coordinates regardless of alliance.
     *
     * @param startId Start position (POS_1, POS_2, POS_3)
     * @param climbSide Selected climb side (LEFT or RIGHT)
     * @return Pose at midpoint in blue coordinates, facing the hub
     */
    public static Pose2d midpointShotPoseBlue(StartPoseId startId, ClimbSide climbSide) {
        Pose2d blueStart = startId == StartPoseId.POS_1 ? BlueLandmarks.Start1
                : (startId == StartPoseId.POS_2 ? BlueLandmarks.Start2 : BlueLandmarks.Start3);
        Pose2d blueTower = (climbSide == ClimbSide.RIGHT) ? BlueLandmarks.TowerAlignRight : BlueLandmarks.TowerAlignLeft;
        Translation2d mid = blueStart.getTranslation().plus(blueTower.getTranslation()).div(2);
        Translation2d hub = BlueLandmarks.Hub.getTranslation();
        Rotation2d towardHub = hub.minus(mid).getAngle();
        return new Pose2d(mid, towardHub);
    }

    /**
     * Test - Climb start pose: 4 ft from tower center toward field center, 4 ft toward closest sideline.
     * Depends on climb side (left → toward top sideline, right → toward bottom sideline).
     */
    public static Pose2d testClimbStart(ClimbSide climbSide) {
        return AllianceUtils.getAlliancePose(BlueLandmarks.testClimbStartBlue(climbSide));
    }

    /**
     * Gets the tower (uprights reference) pose for the current alliance.
     */
    public static Pose2d OurTower() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.Tower);
    }

    /**
     * Gets the tower align pose for the given start. Robot backs into the circle end of the tower bar;
     * Y is chosen so we climb the side of the tower nearest that start (left → high Y, center → mid, right → low Y).
     * Prefer {@link #OurTowerAlignLeft()} / {@link #OurTowerAlignRight()} with a climb-side chooser when center is not usable.
     */
    public static Pose2d OurTowerAlign(StartPoseId startId) {
        if (startId == StartPoseId.POS_1) {
            return AllianceUtils.getAlliancePose(BlueLandmarks.TowerAlignLeft);
        }
        if (startId == StartPoseId.POS_2) {
            return AllianceUtils.getAlliancePose(BlueLandmarks.TowerAlignCenter);
        }
        return AllianceUtils.getAlliancePose(BlueLandmarks.TowerAlignRight);
    }

    /**
     * Tower align pose for climbing on the left side of the bar (high Y in blue).
     * Use when climb side is selected via Shuffleboard; center climb is not physically possible.
     */
    public static Pose2d OurTowerAlignLeft() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.TowerAlignLeft);
    }

    /**
     * Tower align pose for climbing on the right side of the bar (low Y in blue).
     * Use when climb side is selected via Shuffleboard; center climb is not physically possible.
     */
    public static Pose2d OurTowerAlignRight() {
        return AllianceUtils.getAlliancePose(BlueLandmarks.TowerAlignRight);
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
