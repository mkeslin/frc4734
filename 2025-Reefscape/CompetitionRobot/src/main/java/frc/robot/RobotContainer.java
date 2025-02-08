package frc.robot;

import static frc.robot.Constants.Constants.IDs.APRILTAGPIPELINE;
import static frc.robot.Constants.Constants.IDs.INTAKE_SENSOR;
import static frc.robot.Constants.Constants.IDs.REEF_CAMERA_AREA;
import static frc.robot.Constants.Constants.IDs.STATION_CAMERA_AREA;
import static frc.robot.Constants.Constants.IDs.LIGHTS_ID;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.Commands.RobotCommands;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Controllers.ControllerIds;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainA;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainBindings;

public class RobotContainer {

    // CONTROLLERS
    private final CommandXboxController m_driveController = new CommandXboxController(ControllerIds.XC1ID);
    private final CommandXboxController m_mechanismController = new CommandXboxController(ControllerIds.XC2ID);
    private final CommandXboxController m_arcadeController = new CommandXboxController(ControllerIds.XC3ID);

    // PATHPLANNER
    // private final PathPlanner m_pathPlanner = new PathPlanner(m_drivetrain);
    private final SendableChooser<Command> m_autoChooser;

    // private Command runAuto = m_drivetrain.getAutoPath("Auto-1");

    // POSITION TRACKER
    private PositionTracker m_positionTracker = new PositionTracker();

    // DRIVETRAIN
    public final CommandSwerveDrivetrain m_drivetrain = SwerveDrivetrainA.createDrivetrain();

    // SUBSYSTEMS
    private static Limelight m_reef_limelight = new Limelight("limelight-one", APRILTAGPIPELINE);
    private static Limelight m_station_limelight = new Limelight("limelight-two", APRILTAGPIPELINE);
    private Elevator m_elevator = new Elevator(m_positionTracker);
    private Arm m_arm = new Arm(m_positionTracker, m_elevator::getCarriageComponentPose);
    private SideToSide m_sideToSide = new SideToSide(m_positionTracker);
    private Climber m_climber = new Climber(m_positionTracker);
    private Lights m_lights = new Lights();
    private CoralSim m_coralSim = new CoralSim(m_drivetrain::getPose, m_arm::getClawComponentPose);

    private DigitalInput m_intakeSensor = new DigitalInput(INTAKE_SENSOR);

    // COMMANDS
    public CenterToTargetCommand centerToAprilTagCommand = new CenterToTargetCommand(m_reef_limelight, m_drivetrain, REEF_CAMERA_AREA);
    public CenterToTargetCommand centerToStationCommand = new CenterToTargetCommand(m_station_limelight, m_drivetrain, STATION_CAMERA_AREA);

    // AUTO CHOOSERS
    // private final SendableChooser<Integer> m_autoStartChooser = new SendableChooser<>();

    public RobotContainer() {
        // register named commands
        NamedCommands.registerCommand("centerToAprilTagCommand", centerToAprilTagCommand);
        NamedCommands.registerCommand("centerToStationCommand", centerToStationCommand);

        // configure bindings for swerve drivetrain
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);

        // configure bindings for mechanisms
        configureMechanismBindings();

        // configure bindings for arcade/debug
        configureArcadeBindings();

        configureAuto();

        configureDashboard();

        // lights
        // configureLightsBindings();

        // command tests
        // m_driveController.rightBumper().onTrue(acquireNoteCommand);

        // PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser("Auto-1");
        SmartDashboard.putData("Auto Mode - 2025", m_autoChooser);
        // m_pathPlanner.configure();

        // april tags
        // m_mechanismController.a().onTrue(centerToAprilTagCommand);

        // // auto choosers
        // m_autoStartChooser.setDefaultOption("Driver Station 3", 3);
        // m_autoStartChooser.addOption("Driver Station 2", 2);
        // m_autoStartChooser.addOption("Driver Station 1", 1);
        // SmartDashboard.putData("Auto Start Position", m_autoStartChooser);

        // set alliance
        var isRedAlliance = isRedAlliance();
        SwerveDrivetrainBindings.setAllianceOrientation(isRedAlliance);

        m_drivetrain.seedFieldCentric();

        // initialize subsystems
        // new Trigger(() -> {
        // return m_elevator.getInitialized()
        // && m_arm.getInitialized();
        // // && climber.getInitialized();
        // }).onTrue(GlobalStates.INITIALIZED.enableCommand());
        GlobalStates.INITIALIZED.enableCommand();

        // set position
        // resetPose();

        // reset positions
        resetZeros();
    }

    private void configureMechanismBindings() {
        // SIDE-TO-SIDE
        // m_mechanismController.leftTrigger().onTrue(m_sideToSide.moveToSetPositionCommand(() ->
        // SideToSidePosition.LEFT));
        // m_mechanismController.leftBumper().onTrue(m_sideToSide.moveToSetPositionCommand(() ->
        // SideToSidePosition.RIGHT));
        // m_mechanismController.rightBumper().onTrue(m_sideToSide.moveToSetPositionCommand(() ->
        // SideToSidePosition.CENTER));

        // ARM
        // m_mechanismController.leftTrigger().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM));
        // m_mechanismController.leftBumper().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L1));
        // m_mechanismController.rightTrigger().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L2));
        // m_mechanismController.rightBumper().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L4));
        // m_mechanismController.x().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.TOP));

        // ELEVATOR
        // m_mechanismController.leftTrigger().onTrue(m_elevator.moveToSetPositionCommand(() ->
        // ElevatorPosition.BOTTOM));
        // m_mechanismController.leftBumper().onTrue(m_elevator.moveToSetPositionCommand(() -> ElevatorPosition.L1));
        // m_mechanismController.rightTrigger().onTrue(m_elevator.moveToSetPositionCommand(() -> ElevatorPosition.L2));
        // m_mechanismController.rightBumper().onTrue(m_elevator.moveToSetPositionCommand(() -> ElevatorPosition.L3));

        // PREPARE TO SCORE
        // m_mechanismController.rightTrigger().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1,
        //         ScoreSide.Left, m_elevator, m_arm, m_sideToSide, m_coralSim));
        // m_mechanismController.rightBumper().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3,
        //         ScoreSide.Right, m_elevator, m_arm, m_sideToSide, m_coralSim));

        // m_mechanismController.b().onTrue(Commands.run(() -> System.out.printf("mechanism command called")));
        // m_mechanismController.a().onTrue(RobotCommands.scoreCoralCommand(m_drivetrain, m_elevator, m_arm,
        // m_coralSim));

        // m_driveController.leftTrigger().onTrue(Commands.runOnce(() -> resetPose(),
        // m_pathPlanner));

        // GO TO PRE-INTAKE
        m_mechanismController.x()
                .onTrue(RobotCommands.prepareIntakeCoralCommand(m_elevator, m_arm, m_sideToSide, m_coralSim));

        // MOVE TO START POSITION // ALSO INTAKE COMMAND
        m_mechanismController.b().onTrue(RobotCommands.returnToStartPositions(m_elevator, m_arm, m_sideToSide));

        // GO TO POST-INTAKE
        m_mechanismController.y()
                .onTrue(RobotCommands.movePostIntakeCoralCommand(m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));

        // SCORE CORAL
        m_mechanismController.a().onTrue(RobotCommands.scoreCoralCommand(m_drivetrain, m_elevator, m_arm, m_coralSim));

        // SET CURRENT POSITION TO ZERO
        // m_mechanismController.y().onTrue(Commands.runOnce(() -> {
        // m_sideToSide.resetPosition();
        // m_arm.resetPosition();
        // m_elevator.resetPosition();
        // m_climber.resetPosition();
        // }));
    }

    public void configureArcadeBindings() {

        // m_arcadeController.leftTrigger().onTrue(Commands.runOnce(() -> m_elevator.setVoltage(.4)));
        // m_arcadeController.leftBumper().onTrue(Commands.runOnce(() -> m_elevator.setVoltage(0)));

        // m_arcadeController.leftTrigger().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1,
        // ScoreSide.Left, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.rightTrigger().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2,
        // ScoreSide.Left, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.b().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3,
        // ScoreSide.Left, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.a().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4,
        // ScoreSide.Left, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.x().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4,
        // ScoreSide.Right, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.y().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3,
        // ScoreSide.Right, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.rightBumper().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2,
        // ScoreSide.Right, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));
        // m_arcadeController.leftBumper().onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1,
        // ScoreSide.Right, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim));

        // m_arcadeController.start().onTrue(centerToAprilTagCommand);

        m_arcadeController.start().onTrue(Commands.sequence(
            RobotCommands.movePostIntakeCoralCommand(m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim).withTimeout(4),
            centerToAprilTagCommand.withTimeout(4),
            RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, ScoreSide.Right, m_elevator, m_arm, m_sideToSide, m_lights, m_coralSim).withTimeout(4),
            RobotCommands.scoreCoralCommand(m_drivetrain, m_elevator, m_arm, m_coralSim).withTimeout(4),
            RobotCommands.returnToStartPositions(m_elevator, m_arm, m_sideToSide).withTimeout(4),
            Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0)).withTimeout(4)
                    .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0))).asProxy()
        ));

        // m_arcadeController.leftTrigger().onTrue(Commands.run(() -> m_drivetrain.moveRelative(-0.5, 0, 0)).withTimeout(0.35));

        // m_arcadeController.leftTrigger().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.DOWN));
        // m_arcadeController.rightTrigger().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.ACQUIRE));
        // m_arcadeController.b().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.CLIMB));

        // m_arcadeController.a().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM));
        // m_arcadeController.b().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L1));
        // m_arcadeController.rightTrigger().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L2));
        // m_arcadeController.leftTrigger().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L3));
        // m_arcadeController.x().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.L4));
        // m_arcadeController.y().onTrue(m_arm.moveToSetPositionCommand(() -> ArmPosition.TOP));

        // m_arcadeController.a().onTrue(m_intake.commandDeploy());
        // m_arcadeController.x().onTrue(m_intake.commandStow());

        // m_arcadeController.b().onTrue(m_climber.CommandFullRetract());
        // m_arcadeController.y().onTrue(m_climber.CommandFullExtend());

        // m_arcadeController.rightTrigger().onTrue(m_elevator.CommandFullRetract());
        // m_arcadeController.rightBumper().onTrue(m_elevator.CommandFullExtend());

        // m_arcadeController.leftTrigger().onTrue(m_elevator.CommandPivotStow());
        // m_arcadeController.leftBumper().onTrue(m_elevator.CommandPivotDeploy());

        // m_arcadeController.axisLessThan(ControllerButtons.CLY,
        // -0.5).onTrue(shootAmpNoteCommand);
        // m_arcadeController.axisLessThan(ControllerButtons.CLY,
        // -0.5).onTrue(Commands.sequence(shootNoteCommand,
        // m_shooter.commandSetAngle(1)));
        // m_arcadeController
        // .axisGreaterThan(ControllerButtons.CLY, 0.5)
        // .onTrue(intakeNoteCommand);

        // m_arcadeController.axisLessThan(ControllerButtons.CRY,
        // -0.5).onTrue(m_shooter.commandSetAngle(Shooter.TELEOP_SPEAKER_PIVOT_ENCODER_VAL));
        // m_arcadeController.axisGreaterThan(ControllerButtons.CRY,
        // 0.5).onTrue(m_shooter.commandSetAngle(0));
        // m_arcadeController.start().onTrue(m_shooter.commandSetAngle(Shooter.TELEOP_SPEAKER_PIVOT_ENCODER_VAL));

        // m_arcadeController.start().onTrue(Commands.runOnce(() ->
        // m_lights.incrementAnimation(), m_lights));
    }

    public void configureLightsBindings() {
        // m_lights.setDefaultCommand(
        // m_lights.setColors(
        // (int) (m_driveController.getLeftTriggerAxis() * 255),
        // (int) (m_driveController.getRightTriggerAxis() * 255),
        // (int) (m_driveController.getLeftX() * 255)
        // )
        // );

        // m_driveController.y().onTrue(Commands.runOnce(() ->
        // m_lights.incrementAnimation(), m_lights));
    }

    // public void localizeRobotPose() {
    // boolean doRejectUpdate = false;
    // LimelightHelpers.SetRobotOrientation("limelight",
    // // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    // Units.radiansToDegrees(m_drivetrain.getRotation3d().getZ()),
    // 0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // // if our angular velocity is greater than 720 degrees per second, ignore vision
    // // updates
    // if (Math.abs(m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
    // doRejectUpdate = true;
    // }
    // if (!doRejectUpdate) {
    // m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
    // m_drivetrain.addVisionMeasurement(
    // mt2.pose,
    // mt2.timestampSeconds);
    // }
    // }

    public void configureAuto() {
        // AutoManager.getInstance().addRoutine(AutoCommandA.testPath(m_drivetrain));
        // AutoManager.getInstance().addRoutine(AutoCommandA.GDC(m_drivetrain, m_elevator, m_arm, m_coralSim));
    }

    public void configureDashboard() {
        var cmdL1 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, ScoreSide.Left, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score L1", cmdL1);

        var cmdL2 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, ScoreSide.Left, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score L2", cmdL2);

        var cmdL3 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, ScoreSide.Left, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score L3", cmdL3);

        var cmdL4 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, ScoreSide.Left, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score L4", cmdL4);

        var cmdR1 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, ScoreSide.Right, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score R1", cmdR1);

        var cmdR2 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, ScoreSide.Right, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score R2", cmdR2);

        var cmdR3 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, ScoreSide.Right, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score R3", cmdR3);

        var cmdR4 = RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, ScoreSide.Right, m_elevator, m_arm,
                m_sideToSide, m_lights, m_coralSim);
        SmartDashboard.putData("Score R4", cmdR4);
    }

    // public void initializeAuto() {
    // // set alliance
    // var isRedAlliance = isRedAlliance();
    // SwerveDrivetrainBindings.setAllianceOrientation(isRedAlliance);

    // m_drivetrain.seedFieldCentric();

    // // set position
    // resetPose();
    // }

    // public Command getAutonomousCommand() {
    // /* Run the path selected from the auto chooser */
    // return m_autoChooser.getSelected();

    // // // get data from choosers
    // // // m_autoStarPosition = m_autoStartChooser.getSelected();
    // // // var firstNote = m_autoFirstNoteChooser.getSelected();
    // // // var secondNote = m_autoSecondNoteChooser.getSelected();
    // // // var thirdNote = m_autoThirdNoteChooser.getSelected();
    // // // m_autoNoteOrder = new int[] { firstNote, secondNote, thirdNote };

    // // // return m_autoChooser.getSelected();
    // // var autoCommand = new AutoCommand(
    // // m_drivetrain,
    // // // m_pathPlanner,
    // // // m_intake,
    // // // m_shooter,
    // // // m_climber,
    // // // //m_elevator,
    // // // m_intakeLimelight,
    // // // m_shooterLimelight,
    // // m_autoNoteOrder,
    // // m_autoStartPosition,
    // // isRedAlliance()
    // // );
    // // return autoCommand;
    // // return runAuto;

    // // return Commands.print("No autonomous command configured");

    // // return new PathPlannerAuto("Example Auto");
    // // Load the path you want to follow using its name in the GUI
    // // var path = PathPlannerPath.fromPathFile("Auto-1");
    // // Create a path following command using AutoBuilder. This will also trigger
    // // event markers.
    // // return AutoBuilder.followPath(path);
    // }

    // public void initializeTest() {
    // // set alliance
    // var isRedAlliance = isRedAlliance();
    // SwerveDrivetrainBindings.setAllianceOrientation(isRedAlliance);

    // m_drivetrain.seedFieldCentric();

    // // zero all mechanisms
    // // m_intake.zero();
    // // m_climber.zero();
    // // m_elevator.zero();
    // // m_shooter.zero();
    // // move everything to starting position
    // // todo

    // // set position
    // resetPose();
    // }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        var isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        return isRedAlliance;
    }

    public void resetZeros() {
        m_sideToSide.resetPosition();
        m_arm.resetPosition();
        m_elevator.resetPosition();
        m_climber.resetPosition();
    }

    // private void resetPose() {
    // Pose2d startingPosition;
    // switch (m_autoStartPosition) {
    // case 3:
    // startingPosition = Landmarks.OurStart3();
    // break;
    // default:
    // case 2:
    // startingPosition = Landmarks.OurStart2();
    // break;
    // case 1:
    // startingPosition = Landmarks.OurStart1();
    // break;
    // }
    // // m_pathPlanner.resetPose(startingPosition);
    // }
}
