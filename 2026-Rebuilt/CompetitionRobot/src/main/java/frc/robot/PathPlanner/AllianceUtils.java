package frc.robot.PathPlanner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility class for alliance-aware coordinate transformations.
 * 
 * <p>The 2026 field is rotationally symmetrical (not mirrored), meaning poses
 * are rotated 180 degrees around the field center rather than reflected.
 * 
 * <p>This class provides methods to:
 * <ul>
 *   <li>Detect the current alliance color</li>
 *   <li>Transform poses from blue alliance coordinates to red alliance coordinates</li>
 *   <li>Transform poses from red alliance coordinates to blue alliance coordinates</li>
 *   <li>Get alliance-aware poses (always returns blue coordinates for blue alliance,
 *       red coordinates for red alliance)</li>
 *   <li>Load the appropriate field layout for the current alliance</li>
 * </ul>
 * 
 * <p>Field dimensions ({@code FIELD_WIDTH}, {@code FIELD_LENGTH}) and the AprilTag layout
 * ({@code AprilTagFields.k2026RebuiltAndymark}) are set for the 2026 Rebuilt (Andymark) field.
 */
public final class AllianceUtils {
    /**
     * PathPlanner / {@link BlueLandmarks} X extent (m): long axis, origin at blue corner, +X toward red.
     * (Named {@code FIELD_WIDTH} historically; do not swap with {@link #FIELD_LENGTH} when indexing poses.)
     */
    private static final double FIELD_WIDTH = 16.540988;

    /**
     * PathPlanner / {@link BlueLandmarks} Y extent (m): short axis across the field width.
     */
    private static final double FIELD_LENGTH = 8.069326;

    /**
     * Rotation center for {@link #blueToRed} in the same frame as {@code Pose2d} from landmarks:
     * {@code getX()} is along {@link #FIELD_WIDTH}, {@code getY()} along {@link #FIELD_LENGTH}.
     * Matches implicit center used by {@link #flipPositionForPathPlanner} ({@code fieldSize/2}).
     */
    private static final double FIELD_CENTER_X = FIELD_WIDTH / 2.0;

    private static final double FIELD_CENTER_Y = FIELD_LENGTH / 2.0;

    private AllianceUtils() {
        // Utility class - prevent instantiation
    }

    /**
     * Gets the current alliance color from DriverStation.
     * 
     * @return The current alliance, or Blue if alliance is not yet determined
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /**
     * Checks if the robot is on the red alliance.
     * 
     * @return true if on red alliance, false if on blue alliance or alliance not determined
     */
    public static boolean isRedAlliance() {
        return getAlliance() == Alliance.Red;
    }

    /**
     * Checks if the robot is on the blue alliance.
     * 
     * @return true if on blue alliance, false if on red alliance or alliance not determined
     */
    public static boolean isBlueAlliance() {
        return getAlliance() == Alliance.Blue;
    }

    /**
     * Transforms a pose from blue alliance coordinates to red alliance coordinates
     * using 180-degree rotation around the field center.
     * 
     * <p>For a rotationally symmetrical field:
     * <ul>
     *   <li>Position: Rotate 180° around field center</li>
     *   <li>Rotation: Add 180° to the heading</li>
     * </ul>
     * 
     * @param bluePose The pose in blue alliance coordinates
     * @return The pose in red alliance coordinates
     */
    public static Pose2d blueToRed(Pose2d bluePose) {
        // Calculate offset from field center
        Translation2d offsetFromCenter = bluePose.getTranslation().minus(
            new Translation2d(FIELD_CENTER_X, FIELD_CENTER_Y)
        );
        
        // Rotate the offset 180 degrees (negate both x and y)
        Translation2d rotatedOffset = new Translation2d(-offsetFromCenter.getX(), -offsetFromCenter.getY());
        
        // Calculate new position by adding rotated offset back to center
        Translation2d redPosition = new Translation2d(FIELD_CENTER_X, FIELD_CENTER_Y).plus(rotatedOffset);
        
        // Rotate heading by 180 degrees
        Rotation2d redRotation = bluePose.getRotation().plus(Rotation2d.fromDegrees(180));
        
        return new Pose2d(redPosition, redRotation);
    }

    /**
     * Transforms a pose from red alliance coordinates to blue alliance coordinates
     * using 180-degree rotation around the field center.
     * 
     * <p>This is the inverse of {@link #blueToRed(Pose2d)}.
     * 
     * @param redPose The pose in red alliance coordinates
     * @return The pose in blue alliance coordinates
     */
    public static Pose2d redToBlue(Pose2d redPose) {
        // Rotating 180 degrees twice returns to original, so redToBlue is the same as blueToRed
        return blueToRed(redPose);
    }

    /**
     * Gets an alliance-aware pose. If on blue alliance, returns the blue pose as-is.
     * If on red alliance, transforms the blue pose to red coordinates.
     * 
     * <p>This is the primary method to use when you have a pose defined in blue
     * alliance coordinates and need it in the current alliance's coordinate system.
     * 
     * @param bluePose The pose in blue alliance coordinates
     * @return The pose in the current alliance's coordinate system
     */
    public static Pose2d getAlliancePose(Pose2d bluePose) {
        if (isRedAlliance()) {
            return blueToRed(bluePose);
        }
        return bluePose;
    }

    /**
     * Gets an alliance-aware pose from a blue pose and a red pose.
     * Returns the appropriate pose based on the current alliance.
     * 
     * @param bluePose The pose in blue alliance coordinates
     * @param redPose The pose in red alliance coordinates
     * @return The pose for the current alliance
     */
    public static Pose2d getAlliancePose(Pose2d bluePose, Pose2d redPose) {
        if (isRedAlliance()) {
            return redPose;
        }
        return bluePose;
    }

    /**
     * Loads the AprilTag field layout for the current alliance.
     * Uses the 2026 Rebuilt (Andymark) field layout for AprilTag-based pose estimation.
     * 
     * <p>For rotationally symmetrical fields, the same field layout is used for
     * both alliances, but poses must be transformed using the rotation methods.
     * 
     * @return The AprilTag field layout for the current game
     */
    public static AprilTagFieldLayout getFieldLayout() {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }

    /**
     * Gets the field layout constant name for logging/debugging purposes.
     * 
     * @return A string representation of the field layout being used
     */
    public static String getFieldLayoutName() {
        return "k2026RebuiltAndymark";
    }

    /**
     * PathPlanner field dimensions (from navgrid.json). PathPlanner uses X as the long
     * dimension (16.54m toward red), Y as the short (8.07m). Distinct from AllianceUtils
     * FIELD_LENGTH/FIELD_WIDTH which use different axis conventions for Landmarks.
     */
    private static final double PATHPLANNER_FIELD_X = 16.540988;
    private static final double PATHPLANNER_FIELD_Y = 8.069326;

    /**
     * Flips a position from blue side to red side in PathPlanner's blue-origin frame.
     * Uses 180° rotation around field center for the 2026 rotationally symmetrical field.
     * Use when flipping path points for red alliance.
     *
     * @param position Position in blue frame (PathPlanner convention: X toward red, Y width)
     * @return The red-side equivalent position in blue frame
     */
    public static Translation2d flipPositionForPathPlanner(Translation2d position) {
        return new Translation2d(
                PATHPLANNER_FIELD_X - position.getX(),
                PATHPLANNER_FIELD_Y - position.getY());
    }

    /**
     * Flips a rotation for the red side (add 180°).
     *
     * @param rotation Rotation in blue frame
     * @return The flipped rotation for red side
     */
    public static Rotation2d flipRotationForPathPlanner(Rotation2d rotation) {
        return rotation.plus(Rotation2d.fromDegrees(180));
    }
}
