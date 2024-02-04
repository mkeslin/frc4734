package frc.robot;

import static frc.robot.Constants.Constants.APRILTAGPIPELINE;
import static frc.robot.Constants.Constants.NOTEPIPELINE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.AutoCommand;
import frc.robot.Commands.AcquireNoteCommand;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.Commands.ShootSpeakerCommand;
import frc.robot.Controllers.ControllerIds;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LimelightAligner;
import frc.robot.Subsystems.Shooter;
import frc.robot.SwerveDrivetrain.*;

public class RobotContainer {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DRIVETRAIN
    private final CommandSwerveDrivetrain m_drivetrain = SwerveDrivetrainA.DriveTrain;
    // private final CommandSwerveDrivetrain drivetrain = SwerveDrivetrainB.DriveTrain;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // CONTROLLERS
    private final CommandXboxController m_driveController = new CommandXboxController(ControllerIds.XC1ID);
    private final CommandXboxController m_mechanismController = new CommandXboxController(ControllerIds.XC2ID);
    // private final CommandXboxController m_arcadeController = new CommandXboxController(ControllerIds.XC3ID);

    // PATHPLANNER
    private final PathPlanner m_pathPlanner = new PathPlanner(m_drivetrain);
    private final SendableChooser<Command> m_autoChooser;

    // SUBSYSTEMS
    private Limelight m_shooterLimelight = new Limelight("limelight-one", APRILTAGPIPELINE);
    private Limelight m_intakeLimelight = new Limelight("limelight-two", NOTEPIPELINE);
    private Intake m_intake = new Intake();
    private Shooter m_shooter = new Shooter();
    private LimelightAligner m_limelightAligner = new LimelightAligner(m_shooterLimelight, m_intakeLimelight, m_pathPlanner);

    // private TelescopeArm telescopeArm = new TelescopeArm();
    // private Elevator horizontalElevator;
    // private Elevator verticalElevator;

    public RobotContainer() {
        // initialize subsystems
        // horizontalElevator =
        //     new Elevator("horizontal", HORELEVATOR1ID, HORELEVATOR2ID, -200, -14000, -24000);
        // verticalElevator =
        //     new Elevator("vertical", VERTELEVATOR1ID, VERTELEVATOR2ID, 3000, 45000, 45000);

        var acquireNoteCommand = new AcquireNoteCommand(m_shooterLimelight, m_intakeLimelight, m_pathPlanner, m_intake, m_limelightAligner);
        var centerIntakeToTargetCommand = new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, 0);
        var centerShooterToTargetCommand = new CenterToTargetCommand(m_shooterLimelight, m_pathPlanner, 0);

        // Register Named Commands
        NamedCommands.registerCommand(
            "acquireNote",
            acquireNoteCommand
            // intake.commandStartIn().andThen(Commands.waitSeconds(2).andThen(intake.commandStop()))
        );
        NamedCommands.registerCommand(
            "centerIntakeToTargetCommand",
            centerIntakeToTargetCommand
        );
        NamedCommands.registerCommand(
            "centerShooterToTargetCommand",
            centerShooterToTargetCommand
        );
        NamedCommands.registerCommand(
            "shootSpeakerNote",
            new ShootSpeakerCommand(m_shooterLimelight, m_intakeLimelight, m_pathPlanner, m_shooter, m_limelightAligner)
        );
        // var acquireNoteCommand = new AcquireNoteCommand(limelight, pathPlanner, intake, limelightAligner);
        // NamedCommands.registerCommand("acquireNote", acquireNoteCommand.schedule());
        // NamedCommands.registerCommand("shootAmpNote", new ShootAmpCommand(null, pathPlanner, intake, aprilTagAligner));
        // NamedCommands.registerCommand("shootTrapNote", new ShootTrapCommand(null, pathPlanner, intake, aprilTagAligner));

        // configure bindings
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);
        // HorizontalElevatorBindings.configureBindings(mechanismController, horizontalElevator);
        // VerticalElevatorBindings.configureBindings(mechanismController, verticalElevator);

        // intake
        // driveController.y().onTrue(intake.isOn() ? intake.commandStop() : intake.commandStartIn());
        m_driveController.y().onTrue(m_intake.commandStartIn());
        m_driveController.x().onTrue(m_intake.commandStop());

        // m_driveController.rightBumper().onTrue(acquireNoteCommand);

        // shooter
        // driveController.rightBumper().onTrue(shooter.commandShoot());
        // driveController.leftBumper().onTrue(shooter.commandStop());

        // pathplanner buttons
        // m_driveController.x().onTrue(m_pathPlanner.moveToOurSource());

        // m_arcadeController.x().onTrue(m_pathPlanner.moveToOurSource());
        // m_arcadeController.y().onTrue(m_pathPlanner.moveToOurStage1());
        // m_arcadeController.rightBumper().onTrue(m_pathPlanner.moveToOurSpeaker());
        // m_arcadeController.leftBumper().onTrue(m_pathPlanner.moveToOurAmp());

        // m_arcadeController.a().onTrue(m_pathPlanner.moveToOurRing3());
        // m_arcadeController.b().onTrue(m_pathPlanner.moveToOurRing2());
        // m_arcadeController.rightTrigger().onTrue(m_pathPlanner.moveToOurRing1());

        // PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser("Auto-1");
        SmartDashboard.putData("Auto Mode", m_autoChooser);
        m_pathPlanner.configure();

        // note alignment
        m_mechanismController.a().onTrue(centerIntakeToTargetCommand);
        //m_mechanismController.a().onTrue(m_limelightAligner.alignToNote());
        //m_mechanismController.b().onTrue(m_limelightAligner.alignToTag(1));
    }

    public Command getAutonomousCommand() {
        // return m_autoChooser.getSelected();
        var autoCommand = new AutoCommand(m_pathPlanner, m_intake, m_limelightAligner);
        return autoCommand;

        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("Example Auto");
        // Load the path you want to follow using its name in the GUI
        // var path = PathPlannerPath.fromPathFile("Auto-1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        // return AutoBuilder.followPath(path);
    }
}
