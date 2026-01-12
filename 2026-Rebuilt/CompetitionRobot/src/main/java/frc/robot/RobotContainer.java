package frc.robot;

import static frc.robot.Constants.DigitalInputIds.CORAL_ARM_SENSOR;
import static frc.robot.Constants.DigitalInputIds.CORAL_TRAY_SENSOR;
import static frc.robot.Constants.VisionConstants.APRILTAG_PIPELINE;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.AutoCommandA;
import frc.robot.Auto.AutoManager;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.RobotCommands;
import frc.robot.Commands.RobotContext;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Controllers.ControllerIds;
import frc.robot.State.StateMachine;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
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

    // DRIVETRAIN
    public final CommandSwerveDrivetrain m_drivetrain = SwerveDrivetrainA.createDrivetrain();

    // SUBSYSTEMS
    private static Limelight m_reef_limelight = new Limelight("limelight", APRILTAG_PIPELINE);
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final SideToSide m_sideToSide;
    private final Climber m_climber;
    private final Lights m_lights;

    private final DigitalInput m_coralTraySensor = new DigitalInput(CORAL_TRAY_SENSOR);
    private final DigitalInput m_coralArmSensor = new DigitalInput(CORAL_ARM_SENSOR);

    // POSITION TRACKER - created in constructor after subsystems are initialized
    private final PositionTracker m_positionTracker;

    // STATE MACHINE
    private final StateMachine m_stateMachine;

    // ROBOT CONTEXT - contains all common dependencies for commands
    private final RobotContext m_robotContext;

    // COMMANDS
    public CenterToReefCommand m_centerToReefCommand;

    public RobotContainer() {
        // Create subsystems (PositionTracker will be set after creation)
        m_elevator = new Elevator();
        m_arm = new Arm(m_elevator::getCarriageComponentPose);
        m_sideToSide = new SideToSide();
        m_climber = new Climber();
        m_lights = new Lights();

        // Create PositionTracker with all actual suppliers
        // Method references will work correctly since subsystems are now created
        m_positionTracker = new PositionTracker(
                m_elevator::getPosition,
                m_arm::getPosition,
                m_sideToSide::getPosition,
                m_climber::getPosition,
                m_coralTraySensor::get,
                m_coralArmSensor::get,
                () -> 0.0  // algaeIntake not used
        );

        // Set PositionTracker on all subsystems
        // This ensures all subsystems share the same PositionTracker instance with real suppliers,
        // allowing them to query each other's state correctly
        m_elevator.setPositionTracker(m_positionTracker);
        m_arm.setPositionTracker(m_positionTracker);
        m_sideToSide.setPositionTracker(m_positionTracker);
        m_climber.setPositionTracker(m_positionTracker);

        m_centerToReefCommand = new CenterToReefCommand(m_reef_limelight, m_drivetrain,
                m_driveController, 3);

        // register named commands
        NamedCommands.registerCommand("centerToReefCommand", m_centerToReefCommand);

        // configure bindings for swerve drivetrain
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);

        // create state machine (loads states automatically in constructor)
        m_stateMachine = new StateMachine();

        // create robot context with all dependencies
        m_robotContext = new RobotContext(
                m_stateMachine,
                m_positionTracker,
                m_drivetrain,
                m_elevator,
                m_arm,
                m_sideToSide,
                m_lights,
                m_reef_limelight);

        // configure bindings for mechanisms
        configureMechanismBindings();

        // configure bindings for arcade
        configureArcadeBindings();

        // configure bindings for drive
        configureDriveBindings();

        // auto
        configureAuto();

        m_drivetrain.seedFieldCentric();

        // initialize subsystems
        GlobalStates.INITIALIZED.enableCommand();

        // reset positions
        resetZeros();
    }

    private void configureMechanismBindings() {
        // GO TO PRE-INTAKE
        m_mechanismController.a()
                .onTrue(RobotCommands.preIntakeCoralCommand(m_robotContext));

        // INTAKE & POST-INTAKE
        m_mechanismController.b().onTrue(
                Commands.sequence(
                        RobotCommands.intakeCoralCommand(m_robotContext),
                        RobotCommands.postIntakeCoralCommand(m_robotContext)
                ));

        // RESET POSE
        m_mechanismController.start()
                .onTrue(RobotCommands.intakeCoralCommand(m_robotContext));
    }

    private void configureDriveBindings() {
        m_driveController.leftTrigger().onTrue(m_centerToReefCommand);

        // CLIMBER
        m_driveController.y().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.ACQUIRE));
        m_driveController.x().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.CLIMB));

        m_driveController.povRight().whileTrue(Commands.run(() -> m_climber.setVoltage(-1.75)));
        m_driveController.povRight().onFalse(Commands.run(() -> m_climber.setVoltage(0.0)));

        m_driveController.povLeft().whileTrue(Commands.run(() -> m_climber.setVoltage(-3.5)));
        m_driveController.povLeft().onFalse(Commands.run(() -> m_climber.setVoltage(0.0)));
    }

    private Command prepareScoreCoralAndCenterToReefCommand(ScoreLevel scoreLevel, ScoreSide scoreSide,
            boolean centerToReef) {
        var centerToReefCommand = NamedCommands.getCommand("centerToReefCommand");
        return Commands.parallel(
                Commands
                        .waitSeconds(0.0)
                        .andThen(RobotCommands.prepareScoreCoralCommand(m_robotContext, scoreLevel, scoreSide)),
                Commands
                        .waitSeconds(0.0)
                        .andThen(centerToReefCommand.unless(() -> !centerToReef))
        );
    }

    public void configureArcadeBindings() {
        var centerToReef = true;

        m_arcadeController.rightTrigger().onTrue(Commands.sequence(
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L2, ScoreSide.Left, centerToReef),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                        .withTimeout(0.15)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy()
        //
        ));
        m_arcadeController.b()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Left, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.12)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                //
                ));
        m_arcadeController.a()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L4, ScoreSide.Left, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.17)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                //
                ));
        m_arcadeController.x()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L4, ScoreSide.Right, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.15)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                //
                ));
        m_arcadeController.y()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Right, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.12)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                //
                ));
        m_arcadeController.rightBumper().onTrue(Commands.sequence(
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L2, ScoreSide.Right, centerToReef),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                        .withTimeout(0.17)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy()
        //
        ));

        m_arcadeController.start().onTrue(
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Center, centerToReef));

        var scoreCommand = Commands.sequence(
                // move arm down
                RobotCommands.scoreCoralCommand(m_robotContext),
                // move forward to set coral if not completely placed
                Commands.run(() -> m_drivetrain.setRelativeSpeed(0.75, 0, 0)).withTimeout(0.12)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy(),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(-1.0, 0, 0)).withTimeout(0.35)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy(),
                RobotCommands.preIntakeCoralCommand(m_robotContext)
                        .onlyIf(() -> !m_positionTracker.getCoralInArm()),
                RobotCommands.prepareScoreCoralRetryCommand(m_robotContext)
                        .onlyIf(() -> m_positionTracker.getCoralInArm())
        );
        m_arcadeController.leftBumper().onTrue(scoreCommand);
        m_arcadeController.leftTrigger().onTrue(scoreCommand);
    }

    public void localizeRobotPose() {
    }

    public void configureAuto() {
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition1(m_robotContext, m_centerToReefCommand));
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition2(m_robotContext, m_centerToReefCommand));
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition3(m_robotContext, m_centerToReefCommand));

        SmartDashboard.putData("Auto Mode (manager)", AutoManager.getInstance().chooser);
    }

    public void resetZeros() {
        m_sideToSide.resetPosition();
        m_arm.resetPosition();
        m_elevator.resetPosition();
        m_climber.resetPosition();
    }
}
