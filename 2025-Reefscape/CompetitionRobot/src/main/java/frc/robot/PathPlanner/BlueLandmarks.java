package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueLandmarks {

    public static final Pose2d Start1 = getPose(7.3, 6.17, 180);
    public static final Pose2d Start2 = getPose(7.3, 4.02, 180);
    public static final Pose2d Start3 = getPose(7.3, 1.90, 180);
    
    public static final Pose2d CoralStation1 = getPose(1.20, 7.00, 125.0);
    public static final Pose2d CoralStation2 = getPose(1.20, 1.16, -125);

    public static final Pose2d Reef1 = getPose(5.32, 5.50, -120.0);
    public static final Pose2d Reef2 = getPose(6.13, 4.00, 180.0);
    public static final Pose2d Reef3 = getPose(5.32, 2.56, 120.0);
    public static final Pose2d Reef4 = getPose(3.60, 2.56, 60.0);
    public static final Pose2d Reef5 = getPose(2.73, 4.00, 0.0);
    public static final Pose2d Reef6 = getPose(3.60, 5.50, -60.0);

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
