package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueLandmarks {

    public static final Pose2d Start3 = getPose(.75, 6.7, 60);
    public static final Pose2d Start2 = getPose(1.41, 5.55, 0);
    public static final Pose2d Start1 = getPose(.75, 4.39, 300);

    public static final Pose2d Amp = getPose(1.8, 7.5, 90.0);
    public static final Pose2d Speaker = getPose(1.41, 5.55, 0);
    public static final Pose2d Source = getPose(15.3, 1.3, -60.0);

    public static final Pose2d Stage1 = getPose(4.3, 5.0, -60.0);
    public static final Pose2d Stage2 = getPose(5.9, 4.1, 180.0);
    public static final Pose2d Stage3 = getPose(4.3, 3.2, 60.0);

    public static final Pose2d Note3 = getPose(2.17, 7.0, 0.0);
    public static final Pose2d Note2 = getPose(2.17, 5.55, 0.0);
    public static final Pose2d Note1 = getPose(1.97, 4.1, 0.0);
    
    public static final Pose2d Note4 = getPose(2.7, 2.3, 0.0);
    public static final Pose2d Note5 = getPose(7.6, 5.7, 0.0);
    public static final Pose2d Note6 = getPose(7.6, 4.0, 0.0);
    public static final Pose2d Note7 = getPose(7.6, 2.4, 0.0);
    public static final Pose2d Note8 = getPose(7.6, 0.8, 0.0);

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
