package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueLandmarks {

    public static final Pose2d Start1 = getPose(7.3, 6.17, 180);
    public static final Pose2d Start2 = getPose(7.3, 4.02, 180);
    public static final Pose2d Start3 = getPose(7.3, 1.90, 180);

    /** Hub (scoring target) center. Stub: replace with actual 2026 field coordinates. */
    public static final Pose2d Hub = getPose(0, 0, 0);
    /** Tower alignment pose. Stub: replace with actual 2026 field coordinates. */
    public static final Pose2d Tower = getPose(0, 0, 0);

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
