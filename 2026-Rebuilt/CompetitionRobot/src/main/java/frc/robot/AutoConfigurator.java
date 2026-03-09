package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Auto.AutoManager;
import frc.robot.Auto.AutoRoutine;
import frc.robot.Auto.commands.AutoConstants;
import frc.robot.Auto.commands.ClimbSide;
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
import frc.robot.PathPlanner.BlueLandmarks;
import frc.robot.PathPlanner.Landmarks;
import frc.robot.autotest.AutoTestHarness;
import frc.robot.autotest.AutoTestShuffleboard;
import frc.robot.autotest.MoleculeTests;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
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

    /** Chooser for which side of the tower to climb (left or right). Shown on Shuffleboard. */
    private final SendableChooser<ClimbSide> m_climbSideChooser = new SendableChooser<>();

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
            Floor floor,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {
        
        // Start poses evaluated at runtime so alliance is correct when auto runs.
        Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers = new HashMap<>();
        startPoseSuppliers.put(StartPoseId.POS_1, Landmarks::OurStart1);
        startPoseSuppliers.put(StartPoseId.POS_2, Landmarks::OurStart2);
        startPoseSuppliers.put(StartPoseId.POS_3, Landmarks::OurStart3);
        startPoseSuppliers.put(StartPoseId.POS_TEST_CLIMB,
                () -> Landmarks.testClimbStart(ClimbSide.RIGHT));
        
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
        registerAtoms(startPoseSuppliers, vision, shooter, feeder, floor, intake, climber, positionTracker);
        
        // Register molecule tests
        registerMolecules(startPoseSuppliers, vision, shooter, feeder, floor, intake, climber);
        
        // Register full autos
        registerFullAutos(startPoseSuppliers, vision, shooter, feeder, floor, intake, climber, positionTracker);
        
        // Initialize test harness
        m_testHarness.initialize();
        
        // Setup Shuffleboard tab
        AutoTestShuffleboard.setupTab(m_testHarness);
        
        // Publish AutoManager chooser
        SmartDashboard.putData("Auto Mode (manager)", m_autoManager.chooser);

        // Climber auto: choose which side of tower to climb (center not physically possible)
        m_climbSideChooser.setDefaultOption("Climb side: Left", ClimbSide.LEFT);
        m_climbSideChooser.addOption("Climb side: Right", ClimbSide.RIGHT);
        SmartDashboard.putData("Climb Side", m_climbSideChooser);
    }

    /**
     * Registers all atom (single command) tests.
     */
    private void registerAtoms(
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {
        
        StartPoseId defaultPose = StartPoseId.POS_1;
        String defaultPath = MoleculeTests.getPathNameForPose(defaultPose, "StartToShot");
        
        // Seed Odometry
        m_testHarness.registerAtom("SeedOdometry", () -> 
                CmdSeedOdometryFromStartPose.create(defaultPose, startPoseSuppliers, m_drivetrain));
        
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
        
        // Drive To Pose (target in blue for pathfinding)
        m_testHarness.registerAtom("DriveToPose", () -> 
                CmdDriveToPose.create(m_drivetrain, () -> BlueLandmarks.Start1));
        
        // Snap To Heading
        m_testHarness.registerAtom("SnapToHeading", () -> 
                new CmdSnapToHeading(
                        m_drivetrain,
                        () -> m_drivetrain.getPose().getRotation(),
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_HEADING_TIMEOUT));
        
        // Hold Pose
        m_testHarness.registerAtom("HoldPose", () ->
                CmdHoldPose.create(
                        m_drivetrain,
                        () -> m_drivetrain.getPose(),
                        AutoConstants.DEFAULT_POSE_TIMEOUT));
        
        // Shooter commands (on / reverse / off for testing)
        // TEMPORARY: use low speed for low-ceiling room; revert to 3000.0 and 30.0 for competition.
        if (shooter != null) {
            Supplier<Double> testRpm = () -> AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED;
            double testShooterRps = AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED; // RPS (TalonFX velocity = rotations/sec)
            m_testHarness.registerAtom("ShooterOn", () -> 
                    shooter.moveToArbitrarySpeedCommand(() -> testShooterRps));
            m_testHarness.registerAtom("ShooterReverse", () -> 
                    shooter.moveToArbitrarySpeedCommand(() -> -testShooterRps));
            m_testHarness.registerAtom("ShooterOff", () -> 
                    new CmdStopShooter(shooter));
            m_testHarness.registerAtom("ShooterSpinUp", () -> 
                    new CmdShooterSpinUp(shooter, testRpm));
            m_testHarness.registerAtom("WaitShooterAtSpeed", () -> 
                    CmdWaitShooterAtSpeed.create(shooter, testRpm, AutoConstants.TEMPORARY_SHOOTER_TOLERANCE));
            m_testHarness.registerAtom("SetShotMode", () -> 
                    new CmdSetShotMode(shooter, ShotMode.AUTO_SHOT));
            if (feeder != null) {
                m_testHarness.registerAtom("ShootForTime", () -> 
                        CmdShootForTime.create(shooter, feeder, floor, 1.0));
                // Coordinated shoot: wait for shooter at speed, then feed for duration
                m_testHarness.registerAtom("Shoot", () -> 
                        new CmdShootForTime(shooter, feeder, floor, 1.5, true, false, testRpm, 100.0));
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
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
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
                        startPoseSuppliers,
                        pathToShot,
                        AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                        m_drivetrain,
                        vision));
        
        // Molecule 2: Path+SpinUp->Shoot
        if (shooter != null && feeder != null) {
            m_testHarness.registerMolecule("Path+SpinUp->Shoot", () -> 
                    MoleculeTests.buildPathSpinupShoot(
                            pathToShot,
                            AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED,
                            AutoConstants.TEMPORARY_SHOOTER_TOLERANCE,
                            1.0, // Shoot duration
                            m_drivetrain,
                            shooter,
                            feeder,
                            floor));
        }
        
        // Molecule 3: Path->Align->Climb->Hold (align pose in blue for pathfinding)
        if (climber != null) {
            Supplier<Pose2d> alignPoseSupplier = () -> BlueLandmarks.Start1;
            m_testHarness.registerMolecule("Path->Align->Climb->Hold", () -> 
                    MoleculeTests.buildPathAlignClimb(
                            pathToTower,
                            alignPoseSupplier,
                            m_drivetrain,
                            climber));
        }
    }

    /**
     * Registers full autonomous routines.
     */
    private void registerFullAutos(
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {

        // Minimal test auto: seed at A (POS_1), pathfind to B (shot position). Use to verify PathPlanner/AutoBuilder.
        // PathPlanner pathfinding uses the nav grid in blue-origin coordinates; pass blue target so the goal is inside the grid.
        // AutoBuilder flips the path for red alliance when the flip supplier returns true.
        // AutoRoutine.getInitialPose() expects blue; it reflects for Red.
        Command driveAToBTest = Commands.sequence(
                CmdSeedOdometryFromStartPose.create(StartPoseId.POS_1, startPoseSuppliers, m_drivetrain),
                CmdDriveToPose.create(
                        m_drivetrain,
                        () -> BlueLandmarks.ShotPosition,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT))
                .withName("PathPlannerTest-AtoB");
        m_autoManager.addRoutine(new AutoRoutine("Test - PathPlanner", driveAToBTest, List.of(), BlueLandmarks.Start1));

        if (vision == null || shooter == null || feeder == null) {
            return; // Need core subsystems for remaining autos
        }

        StartPoseId defaultPose = StartPoseId.POS_1;
        String pathToShot = MoleculeTests.getPathNameForPose(defaultPose, "StartToShot");

        // Test - Drive and Shoot: seed, path to shot, lower intake, aim, shoot (no center run)
        Command testDriveAndShoot = AutoRoutines.buildTestDriveAndShoot(
                defaultPose,
                startPoseSuppliers,
                pathToShot,
                AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED,
                AutoConstants.TEMPORARY_SHOOTER_TOLERANCE,
                1.0,
                m_drivetrain,
                vision,
                shooter,
                feeder,
                floor,
                intake);
        m_autoManager.addRoutine(new AutoRoutine("Test - Drive and Shoot", testDriveAndShoot, List.of(), BlueLandmarks.Start1));

        // Test - Climb: seed at 4 ft from tower toward center + 4 ft toward sideline, right climb side, drive to tower, extend L1, drive to bar, retract
        if (climber != null) {
            Supplier<Pose2d> towerAlignPoseSupplier = () -> BlueLandmarks.TowerAlignRightOffset;
            Command testClimb = AutoRoutines.buildTestClimb(
                    StartPoseId.POS_TEST_CLIMB,
                    startPoseSuppliers,
                    towerAlignPoseSupplier,
                    m_drivetrain,
                    vision,
                    climber);
            m_autoManager.addRoutine(new AutoRoutine("Test - Climb", testClimb, List.of(),
                    BlueLandmarks.testClimbStartBlue(ClimbSide.RIGHT)));
        }

        // Climber Auto: three start positions; tower side is chosen via Shuffleboard "Climb Side" (center not usable)
        // Tower pose is offset 2 ft toward field center so robot extends climber there, then drives to bar and retracts.
        // Shot and tower poses in blue so pathfinding always gets blue-origin coordinates.
        if (climber != null) {
            Supplier<Pose2d> towerAlignPoseSupplier = () -> {
                ClimbSide side = m_climbSideChooser.getSelected();
                return (side == ClimbSide.RIGHT) ? BlueLandmarks.TowerAlignRightOffset : BlueLandmarks.TowerAlignLeftOffset;
            };
            for (StartPoseId startId : new StartPoseId[] { StartPoseId.POS_1, StartPoseId.POS_2, StartPoseId.POS_3 }) {
                String label = startId == StartPoseId.POS_1 ? "Left" : (startId == StartPoseId.POS_2 ? "Middle" : "Right");
                // Blue coordinates so pathfinding always gets a target inside the nav grid.
                Supplier<Pose2d> shotPoseSupplier = () -> Landmarks.midpointShotPoseBlue(startId, m_climbSideChooser.getSelected());
                Command climberAuto = AutoRoutines.buildClimberAuto(
                        startId,
                        startPoseSuppliers,
                        shotPoseSupplier,
                        towerAlignPoseSupplier,
                        AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                        AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED,
                        AutoConstants.TEMPORARY_SHOOTER_TOLERANCE,
                        1.0, // Shoot duration
                        m_drivetrain,
                        vision,
                        shooter,
                        feeder,
                        floor,
                        intake,
                        climber);
                Pose2d initialPoseBlue = startId == StartPoseId.POS_1 ? BlueLandmarks.Start1
                        : (startId == StartPoseId.POS_2 ? BlueLandmarks.Start2 : BlueLandmarks.Start3);
                m_autoManager.addRoutine(new AutoRoutine(
                        "ClimberAuto (" + label + ")",
                        climberAuto,
                        List.of(),
                        initialPoseBlue));
            }
        }
        
        // Shooter Auto: Left and Right (shoot → through center → return to shot → shoot)
        if (intake != null) {
            for (StartPoseId startId : new StartPoseId[] { StartPoseId.POS_1, StartPoseId.POS_3 }) {
                String label = startId == StartPoseId.POS_1 ? "Left" : "Right";
                String shooterPathToShot = MoleculeTests.getPathNameForPose(startId, "StartToShot");
                String pathThroughCenter = MoleculeTests.getPathNameForPose(startId, "ThroughCenter");
                String pathThroughCenterReturn = MoleculeTests.getPathNameForPose(startId, "ThroughCenterReturn");
                Command shooterAuto = AutoRoutines.buildShooterAuto(
                        startId,
                        startPoseSuppliers,
                        shooterPathToShot,
                        pathThroughCenter,
                        pathThroughCenterReturn,
                        AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                        AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED,
                        AutoConstants.TEMPORARY_SHOOTER_TOLERANCE,
                        1.0, // Shoot duration
                        m_drivetrain,
                        vision,
                        shooter,
                        feeder,
                        floor,
                        intake);
                Pose2d initialPoseBlue = startId == StartPoseId.POS_1 ? BlueLandmarks.Start1 : BlueLandmarks.Start3;
                m_autoManager.addRoutine(new AutoRoutine("ShooterAuto (" + label + ")", shooterAuto, List.of(), initialPoseBlue));
            }
        }

        // Shooter Auto (Center): C_StartToShot → shoot preload → stop (no center run)
        Command shooterAutoCenter = AutoRoutines.buildTestDriveAndShoot(
                StartPoseId.POS_2,
                startPoseSuppliers,
                "C_StartToShot",
                AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                AutoConstants.TEMPORARY_SHOOTER_TARGET_SPEED,
                AutoConstants.TEMPORARY_SHOOTER_TOLERANCE,
                1.0,
                m_drivetrain,
                vision,
                shooter,
                feeder,
                floor,
                intake);
        m_autoManager.addRoutine(new AutoRoutine("ShooterAuto (Center)", shooterAutoCenter, List.of(), BlueLandmarks.Start2));
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
