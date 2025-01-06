package frc.robot.PathPlanner.AprilTags;

import edu.wpi.first.wpilibj.DriverStation;

public class AprilTags {

    public static boolean isRedAlliance() {
        var isRedAlliance = DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        return isRedAlliance;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public static int OurSourceRight() {
        return isRedAlliance() ? RedAprilTags.SourceRight : BlueAprilTags.SourceRight;
    }

    public static int OurSourceLeft() {
        return isRedAlliance() ? RedAprilTags.SourceLeft : BlueAprilTags.SourceLeft;
    }

    public static int OurSpeakerRight() {
        return isRedAlliance() ? RedAprilTags.SpeakerRight : BlueAprilTags.SpeakerRight;
    }

    public static int OurSpeakerLeft() {
        return isRedAlliance() ? RedAprilTags.SpeakerLeft : BlueAprilTags.SpeakerLeft;
    }

    public static int OurAmp() {
        return isRedAlliance() ? RedAprilTags.Amp : BlueAprilTags.Amp;
    }

    public static int OurStage1() {
        return isRedAlliance() ? RedAprilTags.Stage1 : BlueAprilTags.Stage1;
    }

    public static int OurStage2() {
        return isRedAlliance() ? RedAprilTags.Stage2 : BlueAprilTags.Stage2;
    }

    public static int OurStage3() {
        return isRedAlliance() ? RedAprilTags.Stage3 : BlueAprilTags.Stage3;
    }
}
