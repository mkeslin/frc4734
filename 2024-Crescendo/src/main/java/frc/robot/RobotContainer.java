package frc.robot;

import static frc.robot.Constants.Constants.APRILTAGPIPELINE;
import static frc.robot.Constants.Constants.NOTEPIPELINE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
import frc.robot.Commands.SequenceCommands.ShootAmpCommand;
import frc.robot.Commands.SequenceCommands.ShootSpeakerCommand;
import frc.robot.Commands.SequenceCommands.ShootTrapCommand;
import frc.robot.Controllers.ControllerButtons;
import frc.robot.Controllers.ControllerIds;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LimelightAligner;
import frc.robot.Subsystems.Shooter;
import frc.robot.SwerveDrivetrain.*;

public class RobotContainer {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DRIVETRAIN
    public final CommandSwerveDrivetrain m_drivetrain = SwerveDrivetrainA.DriveTrain;
    // private final CommandSwerveDrivetrain drivetrain = SwerveDrivetrainB.DriveTrain;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // CONTROLLERS
    private final CommandXboxController m_driveController = new CommandXboxController(ControllerIds.XC1ID);
    private final CommandXboxController m_mechanismController = new CommandXboxController(ControllerIds.XC2ID);
    // private final CommandXboxController m_arcadeController = new CommandXboxController(ControllerIds.XC3ID);

    // PATHPLANNER
    private final PathPlanner m_pathPlanner = new PathPlanner(m_drivetrain);
    private final SendableChooser<Command> m_autoChooser;

    private Command runAuto = m_drivetrain.getAutoPath("Auto-1");

    // SUBSYSTEMS
    private Limelight m_shooterLimelight = new Limelight("limelight-one", APRILTAGPIPELINE);
    private Limelight m_intakeLimelight = new Limelight("limelight-two", NOTEPIPELINE);
    private Intake m_intake = new Intake();
    private Shooter m_shooter = new Shooter();
    private Elevator m_elevator = new Elevator();
    private LimelightAligner m_limelightAligner = new LimelightAligner(m_shooterLimelight, m_intakeLimelight, m_pathPlanner);

    public RobotContainer() {
        // Commands
        var acquireNoteCommand = new AcquireNoteCommand(m_intakeLimelight, m_pathPlanner, m_intake);
        var centerIntakeToTargetCommand = new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, 0);
        var centerShooterToTargetCommand = new CenterToTargetCommand(m_shooterLimelight, m_pathPlanner, 0);
        var shootAmpNoteCommand = new ShootAmpCommand(m_shooterLimelight, m_pathPlanner, m_intake, m_shooter);
        var shootSpeakerNoteCommand = new ShootSpeakerCommand(m_shooterLimelight, m_intakeLimelight, m_pathPlanner, m_intake, m_shooter);
        var shootTrapNoteCommand = new ShootTrapCommand(m_shooterLimelight, m_pathPlanner, m_intake, m_shooter);

        // Register Named Commands
        NamedCommands.registerCommand("acquireNote", acquireNoteCommand);
        NamedCommands.registerCommand("centerIntakeToTargetCommand", centerIntakeToTargetCommand);
        NamedCommands.registerCommand("centerShooterToTargetCommand", centerShooterToTargetCommand);
        NamedCommands.registerCommand("shootAmpNoteCommand", shootAmpNoteCommand);
        NamedCommands.registerCommand("shootSpeakerNoteCommand", shootSpeakerNoteCommand);
        NamedCommands.registerCommand("shootTrapNoteCommand", shootTrapNoteCommand);
        // var acquireNoteCommand = new AcquireNoteCommand(limelight, pathPlanner, intake, limelightAligner);
        // NamedCommands.registerCommand("acquireNote", acquireNoteCommand.schedule());

        // configure bindings
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);
        // HorizontalElevatorBindings.configureBindings(mechanismController, horizontalElevator);
        // VerticalElevatorBindings.configureBindings(mechanismController, verticalElevator);

        // intake
        // driveController.y().onTrue(intake.isOn() ? intake.commandStop() : intake.commandStartIn());
        m_driveController.y().onTrue(m_intake.commandStartIn());
        m_driveController.x().onTrue(m_intake.commandStopRoller());

        m_driveController.leftBumper().onTrue(m_intake.commandStow());
        m_driveController.leftTrigger().onTrue(m_intake.commandDeploy());
        m_driveController.b().onTrue(m_intake.commandStopPivot());

        // shooter
        m_driveController.y().onTrue(m_shooter.commandShoot());
        m_driveController.x().onTrue(m_shooter.commandStop());

        // elevator
        m_driveController.rightBumper().onTrue(m_elevator.CommandPivotStow());
        m_driveController.rightTrigger().onTrue(m_elevator.CommandPivotDeploy());
        m_driveController.b().onTrue(m_elevator.CommandPivotStop());

        // LEFT STICK - Y - ELEVATOR EXTEND/RETRACT
        m_mechanismController.axisLessThan(ControllerButtons.CLY, -0.5).onTrue(m_elevator.CommandExtend());
        m_mechanismController.axisGreaterThan(ControllerButtons.CLY, 0.5).whileTrue(m_elevator.CommandRetract());
        m_driveController.b().onTrue(m_elevator.CommandStopExtendRetract());

        // RIGHT STICK - Y - SHOOTER ANGLE
        m_mechanismController.axisLessThan(ControllerButtons.CRY, -0.5).onTrue(m_shooter.commandSetAngle(13));
        m_mechanismController.axisGreaterThan(ControllerButtons.CRY, 0.5).whileTrue(m_shooter.commandSetAngle(0));

        // RIGHT STICK - X - ELEVATOR ANGLE
        m_mechanismController.axisLessThan(ControllerButtons.CRX, -0.5).onTrue(m_elevator.CommandPivotDeploy());
        m_mechanismController.axisGreaterThan(ControllerButtons.CRX, 0.5).whileTrue(m_elevator.CommandPivotStow());

        // commands
        // m_driveController.rightBumper().onTrue(acquireNoteCommand);

        // pathplanner test
        // Command testSequence = Commands.sequence(
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! move"),
        //     m_pathPlanner.moveToOurSource(),
        //     // Commands.runOnce(() -> m_pathPlanner.moveToPose(new Pose2d(0, 0, new Rotation2d(0))).schedule()),
        //     // m_pathPlanner.moveToPose(new Pose2d(50,50, new Rotation2d(0))),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake start"),
        //     m_intake.commandStartIn(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake stop"),
        //     m_intake.commandStopRoller()
        // );

        // Command autoSequence = Commands.sequence(
        //     // m_pathPlanner.moveToOurSource()
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! move to first note"),
        //     Commands.runOnce(() -> m_pathPlanner.moveToOurRing1().schedule()),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake start"),
        //     m_intake.commandStartIn(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake stop"),
        //     m_intake.commandStopRoller(),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! shooter start"),
        //     m_shooter.commandShoot(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! shooter stop"),
        //     m_shooter.commandStop(),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! move to first note"),
        //     Commands.runOnce(() -> m_pathPlanner.moveToOurRing2().schedule()),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake start"),
        //     m_intake.commandStartIn(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake stop"),
        //     m_intake.commandStopRoller(),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! shooter start"),
        //     m_shooter.commandShoot(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! shooter stop"),
        //     m_shooter.commandStop(),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! move to first note"),
        //     Commands.runOnce(() -> m_pathPlanner.moveToOurRing3().schedule()),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake start"),
        //     m_intake.commandStartIn(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake stop"),
        //     m_intake.commandStopRoller(),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! shooter start"),
        //     m_shooter.commandShoot(),
        //     Commands.waitSeconds(2),
        //     Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! shooter stop"),
        //     m_shooter.commandStop()
        // );

        /*m_driveController
            .rightBumper()
            .onTrue(
                Commands.sequence(
                    Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! move"),
                    m_pathPlanner.moveToOurSource(),
                    // Commands.runOnce(() -> m_pathPlanner.moveToPose(new Pose2d(0, 0, new Rotation2d(0))).schedule()),
                    // m_pathPlanner.moveToPose(new Pose2d(50,50, new Rotation2d(0))),
                    Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake start"),
                    m_intake.commandStartIn(),
                    Commands.waitSeconds(2),
                    Commands.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! intake stop"),
                    m_intake.commandStopRoller(),

                    m_pathPlanner.moveToOurSpeaker()
                )
            );*/
        // m_driveController.rightBumper().onTrue(m_pathPlanner.moveToOurSource());
        // m_driveController.rightBumper().onTrue(Commands.runOnce(() -> m_pathPlanner.moveToOurSource().schedule()));

        // m_arcadeController.x().onTrue(m_pathPlanner.moveToOurSource());
        // m_arcadeController.y().onTrue(m_pathPlanner.moveToOurStage1());
        // m_arcadeController.rightBumper().onTrue(m_pathPlanner.moveToOurSpeaker());
        // m_arcadeController.leftBumper().onTrue(m_pathPlanner.moveToOurAmp());

        // m_driveController.leftTrigger().onTrue(m_pathPlanner.moveToOurRing2());
        // m_driveController.leftTrigger().onTrue(m_pathPlanner.moveToPose(Landmarks.TheirRing2));
        // m_arcadeController.b().onTrue(m_pathPlanner.moveToOurRing2());
        // m_arcadeController.rightTrigger().onTrue(m_pathPlanner.moveToOurRing1());

        // PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser("Auto-1");
        SmartDashboard.putData("Auto Mode", m_autoChooser);
        m_pathPlanner.configure();

        // note alignment
        m_mechanismController.a().onTrue(acquireNoteCommand);
        m_mechanismController.rightTrigger().onTrue(shootAmpNoteCommand);
        m_mechanismController.y().onTrue(m_shooter.commandSetAngle(13));
        m_mechanismController.x().onTrue(m_shooter.commandSetAngle(0));
        //m_mechanismController.axisLessThan(ControllerButtons.CLY, -0.5).onTrue(m_shooter.commandSetAngle(13));
        //m_mechanismController.axisGreaterThan(ControllerButtons.CLY, 0.5).whileTrue(m_shooter.commandSetAngle(0));
        //m_mechanismController.a().onTrue(m_limelightAligner.alignToNote());
        //m_mechanismController.b().onTrue(m_limelightAligner.alignToTag(1));
    }

    public Command getAutonomousCommand() {
        // return m_autoChooser.getSelected();
        // var autoCommand = new AutoCommand(m_pathPlanner, m_intake, m_limelightAligner);
        // return autoCommand;

        return runAuto;
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("Example Auto");
        // Load the path you want to follow using its name in the GUI
        // var path = PathPlannerPath.fromPathFile("Auto-1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        // return AutoBuilder.followPath(path);
    }
}
