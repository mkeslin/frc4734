package frc.robot;

import static frc.robot.Constants.Constants.APRILTAGPIPELINE;
import static frc.robot.Constants.Constants.NOTEPIPELINE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.AutoCommand;
import frc.robot.Commands.IntakeNoteCommand;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
import frc.robot.Commands.SequenceCommands.ShootAmpCommand;
import frc.robot.Controllers.ControllerButtons;
import frc.robot.Controllers.ControllerIds;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.Shooter;
import frc.robot.SwerveDrivetrain.*;

public class RobotContainer {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DRIVETRAIN
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Drivetrain A is the 2024 robot
    // Drivetrain B is the practice swerve robot
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public final CommandSwerveDrivetrain m_drivetrain = SwerveDrivetrainA.DriveTrain;
    // private final CommandSwerveDrivetrain drivetrain = SwerveDrivetrainB.DriveTrain;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // AUTO NOTE ORDER
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //   |(4)|    | \    (3)  ||
    //   |(5)|    |  >   (2)  || Driver Station
    //   |(6)|    | /    (1)  ||
    //   |(7)|                ||
    //   |(8)|                ||
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public final int[] m_autoNoteOrder = { 1, 2, 3 };
    // public final int[] m_autoNoteOrder = { 6, 7, 8 };
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // CONTROLLERS
    private final CommandXboxController m_driveController = new CommandXboxController(ControllerIds.XC1ID);
    // private final CommandXboxController m_mechanismController = new CommandXboxController(ControllerIds.XC2ID);
    private final CommandXboxController m_arcadeController = new CommandXboxController(ControllerIds.XC3ID);

    // PATHPLANNER
    private final PathPlanner m_pathPlanner = new PathPlanner(m_drivetrain);
    private final SendableChooser<Command> m_autoChooser;

    // private Command runAuto = m_drivetrain.getAutoPath("Auto-1");

    // SUBSYSTEMS
    private Limelight m_shooterLimelight = new Limelight("limelight-one", APRILTAGPIPELINE);
    private Limelight m_intakeLimelight = new Limelight("limelight-two", NOTEPIPELINE);
    private Intake m_intake = new Intake();
    private Shooter m_shooter = new Shooter();
    private Elevator m_elevator = new Elevator();
    private Climber m_climber = new Climber();
    private Lights m_lights = new Lights();

    // private LimelightAligner m_limelightAligner = new LimelightAligner(m_shooterLimelight, m_intakeLimelight, m_pathPlanner);
    // Commands
    public AcquireNoteCommand acquireNoteCommand = new AcquireNoteCommand(m_intakeLimelight, m_pathPlanner, m_intake);
    //public CenterToTargetCommand centerIntakeToTargetCommand = new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, 0);
    //public CenterToTargetCommand centerShooterToTargetCommand = new CenterToTargetCommand(m_shooterLimelight, m_pathPlanner, 0);
    // public ShootAmpCommand shootAmpNoteCommand = new ShootAmpCommand(m_shooterLimelight, m_pathPlanner, m_intake, m_shooter, m_elevator);
    public ShootNoteCommand shootNoteCommand = new ShootNoteCommand(m_intake, m_shooter, 1.0);
    //public ShootSpeakerCommand shootSpeakerNoteCommand = new ShootSpeakerCommand(m_shooterLimelight, m_intakeLimelight, m_pathPlanner, m_intake, m_shooter);
    //public ShootTrapCommand shootTrapNoteCommand = new ShootTrapCommand(m_shooterLimelight, m_pathPlanner, m_intake, m_shooter);
    public IntakeNoteCommand intakeNoteCommand = new IntakeNoteCommand(m_intake);

    public RobotContainer() {
        

        // Register Named Commands
        // NamedCommands.registerCommand("acquireNote", acquireNoteCommand);
        //NamedCommands.registerCommand("centerIntakeToTargetCommand", centerIntakeToTargetCommand);
        //NamedCommands.registerCommand("centerShooterToTargetCommand", centerShooterToTargetCommand);
        // NamedCommands.registerCommand("shootAmpNoteCommand", shootAmpNoteCommand);
        //NamedCommands.registerCommand("shootSpeakerNoteCommand", shootSpeakerNoteCommand);
        //NamedCommands.registerCommand("shootTrapNoteCommand", shootTrapNoteCommand);
        //NamedCommands.registerCommand("intakeNoteCommand", intakeNoteCommand);
        // var acquireNoteCommand = new AcquireNoteCommand(limelight, pathPlanner, intake, limelightAligner);
        // NamedCommands.registerCommand("acquireNote", acquireNoteCommand.schedule());

        // configure bindings for swerve drivetrain
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);

        // configure bindings for mechanisms
        // configureMechanismBindings();

        // configure bindings for arcade/debug
        configureArcadeBindings();

        // lights
        // configureLightsBindings();

        // command tests
        // m_driveController.rightBumper().onTrue(acquireNoteCommand);

        // PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser("Auto-1");
        SmartDashboard.putData("Auto Mode", m_autoChooser);
        m_pathPlanner.configure();

        // note alignment
        // m_mechanismController.a().onTrue(acquireNoteCommand);
        //m_mechanismController.rightTrigger().onTrue(shootAmpNoteCommand);

        m_driveController.y().onTrue(intakeNoteCommand);
        //m_driveController.x().onTrue(shootAmpNoteCommand);
        //m_mechanismController.y().onTrue(m_shooter.commandSetAngle(7.5));
        //m_mechanismController.x().onTrue(m_shooter.commandSetAngle(0));
        //m_mechanismController.axisLessThan(ControllerButtons.CLY, -0.5).onTrue(m_shooter.commandSetAngle(7.5));
        //m_mechanismController.axisGreaterThan(ControllerButtons.CLY, 0.5).whileTrue(m_shooter.commandSetAngle(0));
        //m_mechanismController.a().onTrue(m_limelightAligner.alignToNote());
        //m_mechanismController.b().onTrue(m_limelightAligner.alignToTag(1));
    }

    private void configureMechanismBindings() {
        // intake
        // driveController.y().onTrue(intake.isOn() ? intake.commandStop() : intake.commandStartIn());
        //m_driveController.y().onTrue(m_intake.commandStartIn());
        //m_driveController.x().onTrue(m_intake.commandStopRoller());

        m_driveController.leftBumper().onTrue(m_intake.commandStow());
        m_driveController.leftTrigger().onTrue(m_intake.commandDeploy());
        m_driveController.b().onTrue(m_intake.commandStopPivot());
        // shooter
        //m_driveController.y().onTrue(m_shooter.commandShoot());
        //m_driveController.x().onTrue(m_shooter.commandStop());

        // elevator
        m_driveController.rightBumper().onTrue(m_elevator.CommandPivotStow());
        m_driveController.rightTrigger().onTrue(m_elevator.CommandPivotDeploy());
        m_driveController.b().onTrue(m_elevator.CommandPivotStop());

        // // LEFT STICK - Y - ELEVATOR EXTEND/RETRACT
        // m_mechanismController.leftTrigger().onTrue(m_elevator.CommandFullExtend());
        // m_mechanismController.leftBumper().onTrue(m_elevator.CommandFullRetract());
        // m_driveController.b().onTrue(m_elevator.CommandStopExtendRetract());

        // // RIGHT STICK - Y - SHOOTER ANGLE
        // m_mechanismController.axisLessThan(ControllerButtons.CRY, -0.5).onTrue(m_shooter.commandSetAngle(7.5));
        // m_mechanismController.axisGreaterThan(ControllerButtons.CRY, 0.5).onTrue(m_shooter.commandSetAngle(0));

        // // RIGHT STICK - X - ELEVATOR ANGLE
        // m_mechanismController.axisLessThan(ControllerButtons.CRX, -0.5).onTrue(m_elevator.CommandPivotDeploy());
        // m_mechanismController.axisGreaterThan(ControllerButtons.CRX, 0.5).onTrue(m_elevator.CommandPivotStow());

        // m_mechanismController.x().onTrue(m_climber.CommandFullRetract());
        // m_mechanismController.y().onTrue(m_climber.CommandFullExtend());
    }

    public void configureArcadeBindings() {
        m_arcadeController.a().onTrue(m_intake.commandDeploy());
        m_arcadeController.x().onTrue(m_intake.commandStow());

        m_arcadeController.b().onTrue(m_climber.CommandFullRetract());
        m_arcadeController.y().onTrue(m_climber.CommandFullExtend());

        m_arcadeController.rightTrigger().onTrue(m_elevator.CommandFullRetract());
        m_arcadeController.rightBumper().onTrue(m_elevator.CommandFullExtend());

        m_arcadeController.leftTrigger().onTrue(m_elevator.CommandPivotStow());
        m_arcadeController.leftBumper().onTrue(m_elevator.CommandPivotDeploy());

        // m_arcadeController.axisLessThan(ControllerButtons.CLY, -0.5).onTrue(shootAmpNoteCommand);
        m_arcadeController.axisLessThan(ControllerButtons.CLY, -0.5).onTrue(shootNoteCommand);
        m_arcadeController.axisGreaterThan(ControllerButtons.CLY, 0.5).onTrue(intakeNoteCommand);

        m_arcadeController.axisLessThan(ControllerButtons.CRY, -0.5).onTrue(m_shooter.commandSetAngle(7.5));
        m_arcadeController.axisGreaterThan(ControllerButtons.CRY, 0.5).onTrue(m_shooter.commandSetAngle(0));

        m_arcadeController.start().onTrue(Commands.runOnce(() -> m_lights.incrementAnimation(), m_lights));
    }

    public void configureLightsBindings() {
        m_lights.setDefaultCommand(
            m_lights.setColors(
                (int) (m_driveController.getLeftTriggerAxis() * 255),
                (int) (m_driveController.getRightTriggerAxis() * 255),
                (int) (m_driveController.getLeftX() * 255)
            )
        );

        m_driveController.y().onTrue(Commands.runOnce(() -> m_lights.incrementAnimation(),m_lights));
    }

    public Command getAutonomousCommand() {
        // return m_autoChooser.getSelected();
        // var autoCommand = new AutoCommand(m_pathPlanner, m_intake, m_shooter, m_limelightAligner, m_intakeLimelight, m_shooterLimelight, m_autoNoteOrder);
        var autoCommand = new AutoCommand(m_pathPlanner, m_intake, m_shooter, m_climber, m_elevator, m_intakeLimelight, m_shooterLimelight, m_autoNoteOrder);
        return autoCommand;
        // return runAuto;

        // return Commands.print("No autonomous command configured");

        // return new PathPlannerAuto("Example Auto");
        // Load the path you want to follow using its name in the GUI
        // var path = PathPlannerPath.fromPathFile("Auto-1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        // return AutoBuilder.followPath(path);
    }

    public void initialize() {
        // set alliance
        var alliance = DriverStation.getAlliance();
        SwerveDrivetrainBindings.setAllianceOrientation(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red);

        m_drivetrain.seedFieldRelative();

        // zero all mechanisms
        m_intake.zero();
        m_climber.zero();
        m_elevator.zero();
        
        // move everything to starting position
        // todo

        // set position
        // var startingPosition = new Pose2d(1.25, 5.5, Rotation2d.fromDegrees(0));
        var startingPosition = new Pose2d(15.2, 5.5, Rotation2d.fromDegrees(180));
        m_pathPlanner.resetPose(startingPosition);
    }
}
