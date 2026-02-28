package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Blue alliance field landmarks for 2026 Rebuilt (Andymark) field.
 * All coordinates in meters; derived from official field diagram (inches × 0.0254).
 * Origin: blue alliance corner (bottom-left in diagram); X increases toward red, Y toward opposite wall.
 */
public class BlueLandmarks {

    public static final Pose2d Start1 = getPose(7.3, 6.17, 180);
    public static final Pose2d Start2 = getPose(7.3, 4.02, 180);
    public static final Pose2d Start3 = getPose(7.3, 1.90, 180);

    // ----- Scoring & stage (from diagram, inches → m) -----
    /** Blue diamond plate (scoring target) center. 197.61", 158.845" */
    public static final Pose2d Hub = getPose(5.019, 4.035, 0);
    /** Pose in front of hub for shooting (robot faces hub). Used by drive-to auto. */
    public static final Pose2d ShotPosition = getPose(6.2, 4.035, 180);
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

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
