package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Landmarks {

    // private static boolean m_isRedAlliance = true;
    // private static boolean m_isBlueAlliance = !m_isRedAlliance;

    public static boolean isRedAlliance() {
        var isRedAlliance = DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        return isRedAlliance;
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public static Pose2d OurStartA() {
        return isRedAlliance() ? RedLandmarks.StartA : BlueLandmarks.StartA;
    }

    public static Pose2d OurAmp() {
        return isRedAlliance() ? RedLandmarks.Amp : BlueLandmarks.Amp;
    }

    public static Pose2d OurSpeaker() {
        return isRedAlliance() ? RedLandmarks.Speaker : BlueLandmarks.Speaker;
    }

    public static Pose2d OurSource() {
        return isRedAlliance() ? RedLandmarks.Source : BlueLandmarks.Source;
    }

    public static Pose2d OurStage1() {
        return isRedAlliance() ? RedLandmarks.Stage1 : BlueLandmarks.Stage1;
    }

    public static Pose2d OurStage2() {
        return isRedAlliance() ? RedLandmarks.Stage2 : BlueLandmarks.Stage2;
    }

    public static Pose2d OurStage3() {
        return isRedAlliance() ? RedLandmarks.Stage3 : BlueLandmarks.Stage3;
    }

    public static Pose2d OurNote1() {
        return isRedAlliance() ? RedLandmarks.Note1 : BlueLandmarks.Note1;
    }

    public static Pose2d OurNote2() {
        return isRedAlliance() ? RedLandmarks.Note2 : BlueLandmarks.Note2;
    }

    public static Pose2d OurNote3() {
        return isRedAlliance() ? RedLandmarks.Note3 : BlueLandmarks.Note3;
    }

    public static Pose2d OurNote4() {
        return isRedAlliance() ? RedLandmarks.Note4 : BlueLandmarks.Note4;
    }

    public static Pose2d OurNote5() {
        return isRedAlliance() ? RedLandmarks.Note5 : BlueLandmarks.Note5;
    }

    public static Pose2d OurNote6() {
        return isRedAlliance() ? RedLandmarks.Note6 : BlueLandmarks.Note6;
    }

    public static Pose2d OurNote7() {
        return isRedAlliance() ? RedLandmarks.Note7 : BlueLandmarks.Note7;
    }

    public static Pose2d OurNote8() {
        return isRedAlliance() ? RedLandmarks.Note8 : BlueLandmarks.Note8;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // public static final Pose2d TheirStartA = m_isBlueAlliance ? RedLandmarks.StartA : BlueLandmarks.StartA;

    // public static final Pose2d TheirAmp = m_isBlueAlliance ? RedLandmarks.Amp : BlueLandmarks.Amp;
    // public static final Pose2d TheirSpeaker = m_isBlueAlliance ? RedLandmarks.Speaker : BlueLandmarks.Speaker;
    // public static final Pose2d TheirSource = m_isBlueAlliance ? RedLandmarks.Source : BlueLandmarks.Source;

    // public static final Pose2d TheirStage1 = m_isBlueAlliance ? RedLandmarks.Stage1 : BlueLandmarks.Stage1;
    // public static final Pose2d TheirStage2 = m_isBlueAlliance ? RedLandmarks.Stage2 : BlueLandmarks.Stage2;
    // public static final Pose2d TheirStage3 = m_isBlueAlliance ? RedLandmarks.Stage3 : BlueLandmarks.Stage3;

    // public static final Pose2d TheirRing1 = m_isBlueAlliance ? RedLandmarks.Ring1 : BlueLandmarks.Ring1;
    // public static final Pose2d TheirRing2 = m_isBlueAlliance ? RedLandmarks.Ring2 : BlueLandmarks.Ring2;
    // public static final Pose2d TheirRing3 = m_isBlueAlliance ? RedLandmarks.Ring3 : BlueLandmarks.Ring3;

    // public static final Pose2d TheirRing4 = m_isBlueAlliance ? RedLandmarks.Ring4 : BlueLandmarks.Ring4;
    // public static final Pose2d TheirRing5 = m_isBlueAlliance ? RedLandmarks.Ring5 : BlueLandmarks.Ring5;
    // public static final Pose2d TheirRing6 = m_isBlueAlliance ? RedLandmarks.Ring6 : BlueLandmarks.Ring6;
    // public static final Pose2d TheirRing7 = m_isBlueAlliance ? RedLandmarks.Ring7 : BlueLandmarks.Ring7;
    // public static final Pose2d TheirRing8 = m_isBlueAlliance ? RedLandmarks.Ring8 : BlueLandmarks.Ring8;
}
