package frc.robot;

import static frc.robot.Constants.Constants.IDs.APRILTAGPIPELINE;
import static frc.robot.Constants.Constants.IDs.CORAL_ARM_SENSOR;
import static frc.robot.Constants.Constants.IDs.CORAL_TRAY_SENSOR;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.AutoCommandA;
import frc.robot.Auto.AutoManager;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.CenterToStationCommand;
import frc.robot.Commands.RobotCommands;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
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

    private DigitalInput m_coralTraySensor = new DigitalInput(CORAL_TRAY_SENSOR);
    private DigitalInput m_coralArmSensor = new DigitalInput(CORAL_ARM_SENSOR);

    // COMMANDS
    public CenterToReefCommand m_centerToReefCommand = new CenterToReefCommand(m_reef_limelight, m_drivetrain,
            m_driveController);
    public CenterToStationCommand m_centerToStationCommand = new CenterToStationCommand(m_station_limelight,
            m_drivetrain, m_driveController);

    public RobotContainer() {
        // register named commands
        NamedCommands.registerCommand("centerToReefCommand", m_centerToReefCommand);
        NamedCommands.registerCommand("centerToStationCommand", m_centerToStationCommand);

        // configure bindings for swerve drivetrain
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);

        m_positionTracker.setCoralInTraySupplier(m_coralTraySensor::get);
        m_positionTracker.setCoralInArmSupplier(m_coralArmSensor::get);

        // configure bindings for mechanisms
        configureMechanismBindings();

        // configure bindings for arcade
        configureArcadeBindings();

        // configure bindings for drive
        configureDriveBindings();

        // auto
        configureAuto();

        // lights
        // configureLightsBindings();

        // set alliance
        // var isRedAlliance = isRedAlliance();
        // SwerveDrivetrainBindings.setAllianceOrientation(isRedAlliance);

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
        // GO TO PRE-INTAKE
        m_mechanismController.a()
                .onTrue(RobotCommands.prepareIntakeCoralCommand(m_positionTracker, m_elevator, m_arm, m_sideToSide,
                        m_coralSim));

        // MOVE TO START POSITION // ALSO INTAKE COMMAND
        m_mechanismController.b().onTrue(// Commands.sequence(
                // RobotCommands.prepareIntakeCoralCommand(m_positionTracker, m_elevator, m_arm, m_sideToSide,
                // m_coralSim),
                RobotCommands.returnToStartPositions(m_positionTracker, m_elevator, m_arm, m_sideToSide)
        //
        );

        // m_mechanismController.leftBumper().onTrue(Commands.runOnce(() -> resetZeros()));

        // SET CURRENT POSITION TO ZERO
        // m_mechanismController.y().onTrue(Commands.runOnce(() -> {
        // m_sideToSide.resetPosition();
        // m_arm.resetPosition();
        // m_elevator.resetPosition();
        // m_climber.resetPosition();
        // }));
    }

    private void configureDriveBindings() {
        m_driveController.rightTrigger().onTrue(m_centerToStationCommand);
        m_driveController.leftTrigger().onTrue(m_centerToReefCommand);

        m_driveController.a().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.DOWN));
        m_driveController.b().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.ACQUIRE));
        m_driveController.y().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.CLIMB));
    }

    private Command prepareScoreCoralAndCenterToReefCommand(ScoreLevel scoreLevel, ScoreSide scoreSide, boolean centerToReef) {
        // var centerToReefCommand = new CenterToReefCommand(m_reef_limelight, m_drivetrain, m_driveController);
        var centerToReefCommand = NamedCommands.getCommand("centerToReefCommand");
        return Commands.sequence(
            RobotCommands.prepareScoreCoralCommand(scoreLevel, scoreSide,
                        m_drivetrain, m_elevator, m_arm, m_sideToSide, m_lights, m_reef_limelight, m_coralSim),
            centerToReefCommand.unless(() -> !centerToReef)
        //
        );
    }

    public void configureArcadeBindings() {
        var centerToReef = true;

        m_arcadeController.rightTrigger().onTrue(Commands.sequence(
                RobotCommands.moveIntermediatePrepareScoreCoralCommand(m_elevator, m_arm, m_sideToSide, m_lights,
                        m_coralSim),
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L2, ScoreSide.Left, centerToReef),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(0.5, 0, 0))
                        .withTimeout(0.15)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy()
        //
        ));
        m_arcadeController.b().onTrue(prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Left, centerToReef));
        m_arcadeController.a().onTrue(prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L4, ScoreSide.Left, centerToReef));
        m_arcadeController.x().onTrue(prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L4, ScoreSide.Right, centerToReef));
        m_arcadeController.y().onTrue(prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Right, centerToReef));
        m_arcadeController.rightBumper().onTrue(Commands.sequence(
                RobotCommands.moveIntermediatePrepareScoreCoralCommand(m_elevator, m_arm, m_sideToSide, m_lights,
                        m_coralSim),
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L2, ScoreSide.Right, centerToReef),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(0.5, 0, 0))
                        .withTimeout(0.15)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy()
        //
        ));

        m_arcadeController.leftBumper()
                .onTrue(RobotCommands.scoreCoralCommand(m_drivetrain, m_elevator, m_arm, m_lights, m_coralSim));
        m_arcadeController.leftTrigger()
                .onTrue(RobotCommands.scoreCoralCommand(m_drivetrain, m_elevator, m_arm, m_lights, m_coralSim));

        // LOGGING & SYSID
        // m_arcadeController.leftTrigger().onTrue(Commands.runOnce(SignalLogger::start));
        // m_arcadeController.leftBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // m_arcadeController.a().whileTrue(m_elevator.sysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
        // m_arcadeController.x().whileTrue(m_elevator.sysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
        // m_arcadeController.b().whileTrue(m_elevator.sysIdDynamicCommand(SysIdRoutine.Direction.kForward));
        // m_arcadeController.y().whileTrue(m_elevator.sysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
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

    public void localizeRobotPose() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight-one",
                Units.radiansToDegrees(m_drivetrain.getRotation3d().getZ()),
                0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one");
        if (mt2 == null || mt2.tagCount == 0) {
            return;
        }
        // if our angular velocity is greater than 720 degrees per second, ignore vision
        // updates
        if (Math.abs(m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_drivetrain.addVisionMeasurement(
                    mt2.pose,
                    Utils.fpgaToCurrentTime(mt2.timestampSeconds));
        }
    }

    public void configureAuto() {
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition1(m_positionTracker, m_centerToReefCommand, m_drivetrain,
                        m_elevator, m_arm,
                        m_sideToSide, m_lights, m_reef_limelight, m_station_limelight, m_coralSim));
        // AutoManager.getInstance()
        // .addRoutine(AutoCommandA.StartingPosition2(m_positionTracker, m_drivetrain, m_elevator, m_arm,
        // m_sideToSide, m_lights, m_reef_limelight, m_station_limelight, m_coralSim));
        // AutoManager.getInstance()
        // .addRoutine(AutoCommandA.StartingPosition3(m_positionTracker, m_drivetrain, m_elevator, m_arm,
        // m_sideToSide, m_lights, m_reef_limelight, m_station_limelight, m_coralSim));

        SmartDashboard.putData("Auto Mode (manager)", AutoManager.getInstance().chooser);
    }

    // private boolean isRedAlliance() {
    // var alliance = DriverStation.getAlliance();
    // var isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    // return isRedAlliance;
    // }

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
