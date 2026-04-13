package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Auto.commands.ClimbSide;

/**
 * Blue alliance field landmarks for 2026 Rebuilt (Andymark) field.
 * All coordinates in meters; derived from official field diagram (inches × 0.0254).
 * Origin: blue alliance corner (bottom-left in diagram); X increases toward red, Y toward opposite wall.
 */
public class BlueLandmarks {

    /** Left start: matches L_StartToShot path first waypoint. */
    public static final Pose2d Start1 = getPose(4.1, 7.4, -81.2);
    /** Center start: matches C_StartToShot path first waypoint. */
    public static final Pose2d Start2 = getPose(3.6, 4.035, 0);
    /** Right start: matches R_StartToShot path first waypoint (flipped from left). */
    public static final Pose2d Start3 = getPose(4.1, 0.669326, 81.2);

    // ----- Scoring & stage (from diagram, inches → m) -----
    /** Blue diamond plate (scoring target) center. */
    public static final Pose2d Hub = getPose(4.620, 4.035, 0);
    /** Pose in front of hub for shooting (robot faces hub). Used by drive-to auto. */
    public static final Pose2d ShotPosition = getPose(6.2, 4.035, 180);
    /** Left shooter auto shot position (end of L_StartToShot path). */
    public static final Pose2d ShotPositionLeft = getPose(4.1, 7.4, -81.2);
    /** Center shooter auto shot position (end of C_StartToShot path). */
    public static final Pose2d ShotPositionCenter = getPose(2.6, 4.035, 0);
    /** Right shooter auto shot position (end of R_StartToShot path). */
    public static final Pose2d ShotPositionRight = getPose(4.1, 0.669326, 81.2);
    /** Tower / uprights reference: midpoint between uprights near blue diamond. 312.66", 158.845" */
    public static final Pose2d Tower = getPose(7.942, 4.035, 0);
    /**
     * Pose at tower for climb: back of robot toward circle end of tower bar (looking down the pipe).
     * Left start: climb side nearest start → high Y (between center and top upright).
     */
    public static final Pose2d TowerAlignLeft = getPose(7.6, 4.85, 270);
    /** Center start: climb at bar center. */
    public static final Pose2d TowerAlignCenter = getPose(7.6, 4.035, 270);
    /**
     * Right start: climb side nearest start → low Y (between center and bottom upright).
     */
    public static final Pose2d TowerAlignRight = getPose(7.6, 3.2, 270);

    /**
     * Tower align pose for climb, offset 2 ft toward field center from the bar.
     * Robot extends climber here, then drives 2 ft toward the bar and retracts.
     */
    public static final Pose2d TowerAlignLeftOffset = getPose(.3, 4.597 + 0.5, 90);
    /** Tower align right, offset 2 ft toward field center. */
    public static final Pose2d TowerAlignRightOffset = getPose(1.14 + 0.0, 3.2, 270);

    /** Offset for Test - Climb start: 4 ft from tower center toward center and toward sideline. */
    private static final double TEST_CLIMB_OFFSET_4FT_M = 4 * 0.3048;

    /**
     * Test - Climb start pose: 4 ft from tower center toward field center, 4 ft toward closest sideline.
     * Orientation 270°: back (climber) faces field center vertically, same as tower align pose.
     * Left climb: toward top sideline (+Y). Right climb: toward bottom sideline (-Y).
     */
    public static Pose2d testClimbStartBlue(ClimbSide climbSide) {
        double towerX = Tower.getTranslation().getX();
        double towerY = Tower.getTranslation().getY();
        double x = towerX + TEST_CLIMB_OFFSET_4FT_M;
        double y = climbSide == ClimbSide.LEFT ? towerY + TEST_CLIMB_OFFSET_4FT_M : towerY - TEST_CLIMB_OFFSET_4FT_M;
        return getPose(x, y, 270);
    }

    /** Field center. 523.22", 158.845" */
    public static final Pose2d FieldCenter = getPose(13.290, 4.035, 0);

    /** Blue upright near diamond plate, top. 312.66", 205.845" */
    public static final Pose2d UprightNearHubTop = getPose(7.942, 5.228, 0);
    /** Blue upright near diamond plate, bottom. 312.66", 111.845" */
    public static final Pose2d UprightNearHubBottom = getPose(7.942, 2.841, 0);

    // ----- Feeder / note stacks (blue alliance wall) -----
    /** Blue feeder note stack, bottom center. 29.00", 129.26" */
    public static final Pose2d FeederStackBottom = getPose(0.737, 3.283, 0);
    /** Blue feeder note stack, top center. 29.00", 188.43" */
    public static final Pose2d FeederStackTop = getPose(0.737, 4.786, 0);

    // ----- Alliance station entry openings -----
    /** Blue alliance entry, bottom center. 15.50", 135.465" */
    public static final Pose2d EntryBottom = getPose(0.394, 3.441, 0);
    /** Blue alliance entry, top center. 15.50", 182.225" */
    public static final Pose2d EntryTop = getPose(0.394, 4.629, 0);

    // ----- Center note stack -----
    /** Center note stack center. 523.22", 158.845" */
    public static final Pose2d CenterStackCenter = getPose(13.290, 4.035, 0);

    /**
     * Corral auto (middle): waypoint after center preload shot, before a short +Y run with intake on.
     * Blue-frame coordinates; pathfinding flips on red like other landmarks.
     */
    public static final Pose2d PoseCorralStart = getPose(.84, 5.10, 135);
    /** End pose for corral intake segment (same heading, further +Y). */
    public static final Pose2d PoseCorralStop = getPose(.84, 6.36, 135);

    /** Second shot pose for CorralAuto (middle): return volley after corral pickup. */
    public static final Pose2d PoseAutoShootMiddle = getPose(1.4, 4.035, 0);

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
