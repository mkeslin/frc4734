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
 * <p><b>Field Layout Note:</b> The 2026 field layout constant name is not yet known.
 * This class uses {@code AprilTagFields.kDefaultField} as a placeholder, which should
 * be updated to the specific 2026 constant (e.g., {@code AprilTagFields.k2026RebuiltAndyMark})
 * once the name is confirmed.
 */
public final class AllianceUtils {
    /** Field width in meters (used for rotation calculations) */
    private static final double FIELD_WIDTH = 16.54; // meters
    
    /** Field length in meters (used for rotation calculations) */
    private static final double FIELD_LENGTH = 8.02; // meters
    
    /** Field center X coordinate in meters */
    private static final double FIELD_CENTER_X = FIELD_LENGTH / 2.0;
    
    /** Field center Y coordinate in meters */
    private static final double FIELD_CENTER_Y = FIELD_WIDTH / 2.0;

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
     * 
     * <p><b>Note:</b> The 2026 field layout constant name is not yet known.
     * This method uses {@code AprilTagFields.kDefaultField} as a placeholder.
     * Update this method to use the specific 2026 constant (e.g.,
     * {@code AprilTagFields.k2026RebuiltAndyMark}) once the name is confirmed.
     * 
     * <p>For rotationally symmetrical fields, the same field layout is used for
     * both alliances, but poses must be transformed using the rotation methods.
     * 
     * @return The AprilTag field layout for the current game
     */
    public static AprilTagFieldLayout getFieldLayout() {
        // TODO: Update to use the specific 2026 field constant once the name is known
        // Example: return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndyMark);
        // For now, use kDefaultField which should point to the current game
        return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    /**
     * Gets the field layout constant name for logging/debugging purposes.
     * 
     * @return A string representation of the field layout being used
     */
    public static String getFieldLayoutName() {
        // TODO: Update this when the 2026 constant name is known
        return "kDefaultField (update to 2026 constant when known)";
    }
}
