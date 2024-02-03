package frc.robot.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Landmarks {

    private static boolean m_isRedAlliance = DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    private static boolean m_isBlueAlliance = !m_isRedAlliance;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public static final Pose2d OurStartA = m_isRedAlliance ? RedLandmarks.StartA : BlueLandmarks.StartA;

    public static final Pose2d OurAmp = m_isRedAlliance ? RedLandmarks.Amp : BlueLandmarks.Amp;
    public static final Pose2d OurSpeaker = m_isRedAlliance ? RedLandmarks.Speaker : BlueLandmarks.Speaker;
    public static final Pose2d OurSource = m_isRedAlliance ? RedLandmarks.Source : BlueLandmarks.Source;

    public static final Pose2d OurStage1 = m_isRedAlliance ? RedLandmarks.Stage1 : BlueLandmarks.Stage1;
    public static final Pose2d OurStage2 = m_isRedAlliance ? RedLandmarks.Stage2 : BlueLandmarks.Stage2;
    public static final Pose2d OurStage3 = m_isRedAlliance ? RedLandmarks.Stage3 : BlueLandmarks.Stage3;

    public static final Pose2d OurRing1 = m_isRedAlliance ? RedLandmarks.Ring1 : BlueLandmarks.Ring1;
    public static final Pose2d OurRing2 = m_isRedAlliance ? RedLandmarks.Ring2 : BlueLandmarks.Ring2;
    public static final Pose2d OurRing3 = m_isRedAlliance ? RedLandmarks.Ring3 : BlueLandmarks.Ring3;

    public static final Pose2d OurRing4 = m_isRedAlliance ? RedLandmarks.Ring4 : BlueLandmarks.Ring4;
    public static final Pose2d OurRing5 = m_isRedAlliance ? RedLandmarks.Ring5 : BlueLandmarks.Ring5;
    public static final Pose2d OurRing6 = m_isRedAlliance ? RedLandmarks.Ring6 : BlueLandmarks.Ring6;
    public static final Pose2d OurRing7 = m_isRedAlliance ? RedLandmarks.Ring7 : BlueLandmarks.Ring7;
    public static final Pose2d OurRing8 = m_isRedAlliance ? RedLandmarks.Ring8 : BlueLandmarks.Ring8;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public static final Pose2d TheirStartA = m_isBlueAlliance ? RedLandmarks.StartA : BlueLandmarks.StartA;

    public static final Pose2d TheirAmp = m_isBlueAlliance ? RedLandmarks.Amp : BlueLandmarks.Amp;
    public static final Pose2d TheirSpeaker = m_isBlueAlliance ? RedLandmarks.Speaker : BlueLandmarks.Speaker;
    public static final Pose2d TheirSource = m_isBlueAlliance ? RedLandmarks.Source : BlueLandmarks.Source;

    public static final Pose2d TheirStage1 = m_isBlueAlliance ? RedLandmarks.Stage1 : BlueLandmarks.Stage1;
    public static final Pose2d TheirStage2 = m_isBlueAlliance ? RedLandmarks.Stage2 : BlueLandmarks.Stage2;
    public static final Pose2d TheirStage3 = m_isBlueAlliance ? RedLandmarks.Stage3 : BlueLandmarks.Stage3;

    public static final Pose2d TheirRing1 = m_isBlueAlliance ? RedLandmarks.Ring1 : BlueLandmarks.Ring1;
    public static final Pose2d TheirRing2 = m_isBlueAlliance ? RedLandmarks.Ring2 : BlueLandmarks.Ring2;
    public static final Pose2d TheirRing3 = m_isBlueAlliance ? RedLandmarks.Ring3 : BlueLandmarks.Ring3;

    public static final Pose2d TheirRing4 = m_isBlueAlliance ? RedLandmarks.Ring4 : BlueLandmarks.Ring4;
    public static final Pose2d TheirRing5 = m_isBlueAlliance ? RedLandmarks.Ring5 : BlueLandmarks.Ring5;
    public static final Pose2d TheirRing6 = m_isBlueAlliance ? RedLandmarks.Ring6 : BlueLandmarks.Ring6;
    public static final Pose2d TheirRing7 = m_isBlueAlliance ? RedLandmarks.Ring7 : BlueLandmarks.Ring7;
    public static final Pose2d TheirRing8 = m_isBlueAlliance ? RedLandmarks.Ring8 : BlueLandmarks.Ring8;
}
