package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;

public class Landmarks {

    // public static boolean isRedAlliance() {
    // var isRedAlliance = DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() ==
    // DriverStation.Alliance.Red;
    // return isRedAlliance;
    // }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public static Pose2d OurStart1() {
        // return isRedAlliance() ? RedLandmarks.Start1 : BlueLandmarks.Start1;
        return BlueLandmarks.Start1;
    }

    public static Pose2d OurStart2() {
        // return isRedAlliance() ? RedLandmarks.Start2 : BlueLandmarks.Start2;
        return BlueLandmarks.Start2;
    }

    public static Pose2d OurStart3() {
        // return isRedAlliance() ? RedLandmarks.Start3 : BlueLandmarks.Start3;
        return BlueLandmarks.Start3;
    }

    public static Pose2d CoralStation1() {
        // return isRedAlliance() ? RedLandmarks.Start3 : BlueLandmarks.Start3;
        return BlueLandmarks.CoralStation1;
    }

    public static Pose2d CoralStation2() {
        // return isRedAlliance() ? RedLandmarks.Start3 : BlueLandmarks.Start3;
        return BlueLandmarks.CoralStation2;
    }

    public static Pose2d Reef1() {
        return BlueLandmarks.Reef1;
    }

    public static Pose2d Reef2() {
        return BlueLandmarks.Reef2;
    }

    public static Pose2d Reef3() {
        return BlueLandmarks.Reef3;
    }

    public static Pose2d Reef4() {
        return BlueLandmarks.Reef4;
    }

    public static Pose2d Reef5() {
        return BlueLandmarks.Reef5;
    }

    public static Pose2d Reef6() {
        return BlueLandmarks.Reef6;
    }
}
