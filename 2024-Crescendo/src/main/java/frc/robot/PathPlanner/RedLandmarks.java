package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RedLandmarks {

    public static final Pose2d StartA = getPose(15.2, 5.5, 0);

    public static final Pose2d Amp = getPose(14.8, 7.5, 90.0);
    public static final Pose2d Speaker = getPose(15, 5.5, 180);
    public static final Pose2d Source = getPose(1.3, 1.3, -60.0);

    public static final Pose2d Stage1 = getPose(12.2, 5.0, -60.0);
    public static final Pose2d Stage2 = getPose(10.7, 4.1, 180.0);
    public static final Pose2d Stage3 = getPose(12.2, 3.2, 60.0);

    public static final Pose2d Note1 = getPose(14.3, 7.0, 180.0);
    public static final Pose2d Note2 = getPose(14.3, 5.5, 180.0);
    public static final Pose2d Note3 = getPose(14.3, 4.0, 180.0);

    public static final Pose2d Note4 = getPose(8.9, 7.4, 180.0);
    public static final Pose2d Note5 = getPose(8.9, 5.7, 180.0);
    public static final Pose2d Note6 = getPose(8.9, 4.0, 180.0);
    public static final Pose2d Note7 = getPose(8.9, 2.4, 180.0);
    public static final Pose2d Note8 = getPose(8.9, 0.8, 180.0);

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
