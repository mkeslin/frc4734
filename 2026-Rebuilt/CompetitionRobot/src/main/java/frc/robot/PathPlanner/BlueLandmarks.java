package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueLandmarks {

    public static final Pose2d Start1 = getPose(7.3, 6.17, 180);
    public static final Pose2d Start2 = getPose(7.3, 4.02, 180);
    public static final Pose2d Start3 = getPose(7.3, 1.90, 180);
    
    // TODO: Add 2026 field landmarks (hub, tower, etc.) as needed

    private static Pose2d getPose(double x, double y, double rotationDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
    }
}
