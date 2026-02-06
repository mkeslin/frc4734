package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.AutoManager;
import frc.robot.Auto.AutoRoutine;
import frc.robot.Auto.commands.AutoConstants;
import frc.robot.Auto.commands.AutoRoutines;
import frc.robot.Auto.commands.CmdAcquireHubAim;
import frc.robot.Auto.commands.CmdApplyTagSnapIfGood;
import frc.robot.Auto.commands.CmdClimbL1;
import frc.robot.Auto.commands.CmdDriveToPose;
import frc.robot.Auto.commands.CmdFollowPath;
import frc.robot.Auto.commands.CmdHoldClimbUntilEnd;
import frc.robot.Auto.commands.CmdHoldPose;
import frc.robot.Auto.commands.CmdIntakeOff;
import frc.robot.Auto.commands.CmdIntakeOn;
import frc.robot.Auto.commands.CmdIntakeUntilCount;
import frc.robot.Auto.commands.CmdSeedOdometryFromStartPose;
import frc.robot.Auto.commands.CmdSetShotMode;
import frc.robot.Auto.commands.CmdShootForTime;
import frc.robot.Auto.commands.CmdShooterSpinUp;
import frc.robot.Auto.commands.CmdSnapToHeading;
import frc.robot.Auto.commands.CmdStopAll;
import frc.robot.Auto.commands.CmdStopShooter;
import frc.robot.Auto.commands.CmdWaitShooterAtSpeed;
import frc.robot.Auto.commands.ShotMode;
import frc.robot.Auto.commands.StartPoseId;
import frc.robot.Constants.FeederConstants.FeederSpeed;
import frc.robot.PathPlanner.Landmarks;
import frc.robot.autotest.AutoTestHarness;
import frc.robot.autotest.AutoTestShuffleboard;
import frc.robot.autotest.MoleculeTests;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Configures autonomous routines for the robot.
 * Centralizes autonomous setup logic including test harness registration.
 */
public class AutoConfigurator {
    private final AutoManager m_autoManager;
    private final CommandSwerveDrivetrain m_drivetrain;
    private AutoTestHarness m_testHarness;

    /**
     * Creates a new AutoConfigurator with required dependencies.
     * 
     * @param autoManager The AutoManager instance
     * @param drivetrain The drivetrain subsystem
     */
    public AutoConfigurator(AutoManager autoManager, CommandSwerveDrivetrain drivetrain) {
        m_autoManager = autoManager;
        m_drivetrain = drivetrain;
    }

    /**
     * Configures all autonomous routines and registers them with AutoManager.
     * Also sets up the testing framework with atom, molecule, and full auto tests.
     * 
     * <p>Subsystems may be null during drivetrain-only testing - tests will be
     * registered only if required subsystems are available.
     */
    public void configureAuto(
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {
        
        // Create start poses map
        Map<StartPoseId, Pose2d> startPoses = new HashMap<>();
        startPoses.put(StartPoseId.POS_1, Landmarks.OurStart1());
        startPoses.put(StartPoseId.POS_2, Landmarks.OurStart2());
        startPoses.put(StartPoseId.POS_3, Landmarks.OurStart3());
        
        // Create test harness
        m_testHarness = new AutoTestHarness(
                m_drivetrain,
                vision,
                shooter,
                feeder,
                intake,
                climber,
                positionTracker);
        
        // Register atom commands
        registerAtoms(startPoses, vision, shooter, feeder, intake, climber, positionTracker);
        
        // Register molecule tests
        registerMolecules(startPoses, vision, shooter, feeder, intake, climber);
        
        // Register full autos
        registerFullAutos(startPoses, vision, shooter, feeder, intake, climber, positionTracker);
        
        // Initialize test harness
        m_testHarness.initialize();
        
        // Setup Shuffleboard tab
        AutoTestShuffleboard.setupTab(m_testHarness);
        
        // Publish AutoManager chooser
        SmartDashboard.putData("Auto Mode (manager)", m_autoManager.chooser);
    }

    /**
     * Registers all atom (single command) tests.
     */
    private void registerAtoms(
            Map<StartPoseId, Pose2d> startPoses,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {
        
        StartPoseId defaultPose = StartPoseId.POS_1;
        String defaultPath = MoleculeTests.getPathNameForPose(defaultPose, "StartToShot");
        
        // Seed Odometry
        m_testHarness.registerAtom("SeedOdometry", () -> 
                CmdSeedOdometryFromStartPose.create(defaultPose, startPoses, m_drivetrain));
        
        // Tag Snap
        if (vision != null) {
            m_testHarness.registerAtom("ApplyTagSnap", () -> 
                    new CmdApplyTagSnapIfGood(
                            vision,
                            m_drivetrain,
                            AutoConstants.DEFAULT_MAX_AMBIGUITY,
                            AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                            AutoConstants.DEFAULT_MIN_TARGETS));
        }
        
        // Follow Path
        m_testHarness.registerAtom("FollowPath", () -> 
                new CmdFollowPath(defaultPath, AutoConstants.DEFAULT_PATH_TIMEOUT, m_drivetrain));
        
        // Drive To Pose
        Pose2d testPose = startPoses.get(defaultPose);
        m_testHarness.registerAtom("DriveToPose", () -> 
                CmdDriveToPose.create(m_drivetrain, () -> testPose));
        
        // Snap To Heading
        m_testHarness.registerAtom("SnapToHeading", () -> 
                new CmdSnapToHeading(
                        m_drivetrain,
                        () -> m_drivetrain.getPose().getRotation(),
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_HEADING_TIMEOUT));
        
        // Hold Pose
        m_testHarness.registerAtom("HoldPose", () -> 
                new CmdHoldPose(
                        m_drivetrain,
                        () -> m_drivetrain.getPose(),
                        AutoConstants.DEFAULT_POSE_TIMEOUT));
        
        // Shooter commands (on / reverse / off for testing)
        // Use arbitrary speed in RPS so test is visible; ShooterSpeed.FORWARD (0.5 RPS) is too low to see.
        if (shooter != null) {
            Supplier<Double> testRpm = () -> 3000.0; // Test RPM
            double testShooterRps = 30.0; // RPS for ShooterOn/Reverse (visible; TalonFX velocity = rotations/sec)
            m_testHarness.registerAtom("ShooterOn", () -> 
                    shooter.moveToArbitrarySpeedCommand(() -> testShooterRps));
            m_testHarness.registerAtom("ShooterReverse", () -> 
                    shooter.moveToArbitrarySpeedCommand(() -> -testShooterRps));
            m_testHarness.registerAtom("ShooterOff", () -> 
                    new CmdStopShooter(shooter));
            m_testHarness.registerAtom("ShooterSpinUp", () -> 
                    new CmdShooterSpinUp(shooter, testRpm));
            m_testHarness.registerAtom("WaitShooterAtSpeed", () -> 
                    CmdWaitShooterAtSpeed.create(shooter, testRpm, 100.0));
            m_testHarness.registerAtom("SetShotMode", () -> 
                    new CmdSetShotMode(shooter, ShotMode.AUTO_SHOT));
            if (feeder != null) {
                m_testHarness.registerAtom("ShootForTime", () -> 
                        CmdShootForTime.create(shooter, feeder, 1.0));
                // Coordinated shoot: wait for shooter at speed, then feed for duration
                m_testHarness.registerAtom("Shoot", () -> 
                        new CmdShootForTime(shooter, feeder, 1.5, true, testRpm, 100.0));
            }
            m_testHarness.registerAtom("StopShooter", () -> 
                    new CmdStopShooter(shooter));
        }

        // Feeder commands (on / reverse / off; run until cancelled or FeederOff)
        if (feeder != null) {
            m_testHarness.registerAtom("FeederOn", () -> 
                    feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value));
            m_testHarness.registerAtom("FeederReverse", () -> 
                    feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value));
            m_testHarness.registerAtom("FeederOff", () -> 
                    feeder.resetSpeedCommand());
        }
        
        // Intake commands
        if (intake != null) {
            m_testHarness.registerAtom("IntakeOn", () -> 
                    CmdIntakeOn.create(intake));
            
            m_testHarness.registerAtom("IntakeOff", () -> 
                    new CmdIntakeOff(intake));
            
            if (positionTracker != null) {
                m_testHarness.registerAtom("IntakeUntilCount", () -> 
                        CmdIntakeUntilCount.create(intake, positionTracker, 1));
            }
        }
        
        // Hub Aim
        if (vision != null) {
            m_testHarness.registerAtom("AcquireHubAim", () -> 
                    CmdAcquireHubAim.create(vision, m_drivetrain, 180.0));
        }
        
        // Climb commands
        if (climber != null) {
            m_testHarness.registerAtom("ClimbL1", () -> 
                    CmdClimbL1.create(climber));
            
            m_testHarness.registerAtom("HoldClimbUntilEnd", () -> 
                    new CmdHoldClimbUntilEnd(climber, m_drivetrain));
        }
        
        // Stop All
        m_testHarness.registerAtom("StopAll", () -> 
                CmdStopAll.create(m_drivetrain, shooter, feeder, intake));
    }

    /**
     * Registers all molecule (2-3 command sequence) tests.
     */
    private void registerMolecules(
            Map<StartPoseId, Pose2d> startPoses,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber) {
        
        if (vision == null) {
            return; // Need vision for molecules
        }
        
        StartPoseId defaultPose = StartPoseId.POS_1;
        String pathToShot = MoleculeTests.getPathNameForPose(defaultPose, "StartToShot");
        String pathToTower = MoleculeTests.getPathNameForPose(defaultPose, "StartToTower");
        
        // Molecule 1: Seed->Path->TagSnap->Aim
        m_testHarness.registerMolecule("Seed->Path->TagSnap->Aim", () -> 
                MoleculeTests.buildSeedPathTagSnapAim(
                        defaultPose,
                        startPoses,
                        pathToShot,
                        AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                        m_drivetrain,
                        vision));
        
        // Molecule 2: Path+SpinUp->Shoot
        if (shooter != null && feeder != null) {
            m_testHarness.registerMolecule("Path+SpinUp->Shoot", () -> 
                    MoleculeTests.buildPathSpinupShoot(
                            pathToShot,
                            3000.0, // Test RPM
                            100.0, // RPM tolerance
                            1.0, // Shoot duration
                            m_drivetrain,
                            shooter,
                            feeder));
        }
        
        // Molecule 3: Path->Align->Climb->Hold
        if (climber != null) {
            Pose2d alignPose = startPoses.get(defaultPose); // Use start pose as test align pose
            m_testHarness.registerMolecule("Path->Align->Climb->Hold", () -> 
                    MoleculeTests.buildPathAlignClimb(
                            pathToTower,
                            alignPose,
                            m_drivetrain,
                            climber));
        }
    }

    /**
     * Registers full autonomous routines.
     */
    private void registerFullAutos(
            Map<StartPoseId, Pose2d> startPoses,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {
        
        if (vision == null || shooter == null || feeder == null) {
            return; // Need core subsystems for autos
        }
        
        StartPoseId defaultPose = StartPoseId.POS_1;
        String pathToShot = MoleculeTests.getPathNameForPose(defaultPose, "StartToShot");
        String pathToCenter = MoleculeTests.getPathNameForPose(defaultPose, "StartToCenter");
        String pathBackToShot = MoleculeTests.getPathNameForPose(defaultPose, "CenterToShot");
        String pathToTower = MoleculeTests.getPathNameForPose(defaultPose, "StartToTower");
        Pose2d towerAlignPose = startPoses.get(defaultPose); // Placeholder
        
        // Climber Auto
        if (climber != null) {
            Command climberAuto = AutoRoutines.buildClimberAuto(
                    defaultPose,
                    startPoses,
                    pathToShot,
                    pathToTower,
                    towerAlignPose,
                    AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                    3000.0, // Target RPM
                    100.0, // RPM tolerance
                    1.0, // Shoot duration
                    m_drivetrain,
                    vision,
                    shooter,
                    feeder,
                    climber);
            m_autoManager.addRoutine(new AutoRoutine("ClimberAuto", climberAuto));
        }
        
        // Shooter Auto
        if (intake != null && positionTracker != null) {
            Command shooterAuto = AutoRoutines.buildShooterAuto(
                    defaultPose,
                    startPoses,
                    pathToShot,
                    pathToCenter,
                    pathBackToShot,
                    AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                    3000.0, // Target RPM
                    100.0, // RPM tolerance
                    1.0, // Shoot duration
                    1, // Target ball count
                    m_drivetrain,
                    vision,
                    shooter,
                    feeder,
                    intake,
                    positionTracker);
            m_autoManager.addRoutine(new AutoRoutine("ShooterAuto", shooterAuto));
        }
    }

    /**
     * Gets the test harness for periodic updates.
     * 
     * @return The AutoTestHarness instance, or null if not initialized
     */
    public AutoTestHarness getTestHarness() {
        return m_testHarness;
    }
}
