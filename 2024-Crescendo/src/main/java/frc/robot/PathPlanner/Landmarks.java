package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Landmarks {
    public static final Pose2d StartA = getPose(1.2, 5.5, 0);

    public static final Pose2d Amp = getPose(1.8, 7.5, 90.0);
    public static final Pose2d Speaker = getPose(1.2, 5.5, 180);
    public static final Pose2d Source = getPose(15.3, 1.3, -60.0);

    public static final Pose2d Stage1 = getPose(4.3, 5.0, -60.0);
    public static final Pose2d Stage2 = getPose(5.9, 4.1, 180.0);
    public static final Pose2d Stage3 = getPose(4.3, 3.2, 60.0);

    private static Pose2d getPose(double x, double y, double rotationDegrees)
    {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
