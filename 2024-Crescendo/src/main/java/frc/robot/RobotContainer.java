package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AcquireNoteCommand;
import frc.robot.Commands.ShootSpeakerCommand;
import frc.robot.Controllers.ControllerIds;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.AprilTagAligner;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.SwerveDrivetrain.*;

public class RobotContainer {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DRIVETRAIN
    private final CommandSwerveDrivetrain drivetrain = SwerveDrivetrainA.DriveTrain;
    // private final CommandSwerveDrivetrain drivetrain = SwerveDrivetrainB.DriveTrain;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // CONTROLLERS
    private final CommandXboxController driveController = new CommandXboxController(ControllerIds.XC1ID);
    // private final CommandXboxController mechanismController = new CommandXboxController(
    //     ControllerIds.XC2ID
    // );

    // PATHPLANNER
    private final PathPlanner pathPlanner = new PathPlanner(drivetrain);
    private final SendableChooser<Command> autoChooser;

    // SUBSYSTEMS
    private Intake intake = new Intake();
    private Shooter shooter = new Shooter();
    private AprilTagAligner aprilTagAligner = new AprilTagAligner(null, pathPlanner);
    // private Elevator horizontalElevator;
    // private Elevator verticalElevator;

    public RobotContainer() {
        // initialize subsystems
        // horizontalElevator =
        //     new Elevator("horizontal", HORELEVATOR1ID, HORELEVATOR2ID, -200, -14000, -24000);
        // verticalElevator =
        //     new Elevator("vertical", VERTELEVATOR1ID, VERTELEVATOR2ID, 3000, 45000, 45000);

        // Register Named Commands
        NamedCommands.registerCommand("acquireNote", new AcquireNoteCommand(null, pathPlanner, intake));
        NamedCommands.registerCommand("shootSpeakerNote", new ShootSpeakerCommand(null, pathPlanner, shooter, aprilTagAligner));
        // NamedCommands.registerCommand("shootAmpNote", new ShootAmpCommand(null, pathPlanner, intake, aprilTagAligner));
        // NamedCommands.registerCommand("shootTrapNote", new ShootTrapCommand(null, pathPlanner, intake, aprilTagAligner));

        // configure bindings
        SwerveDrivetrainBindings.configureBindings(driveController, drivetrain);
        // HorizontalElevatorBindings.configureBindings(mechanismController, horizontalElevator);
        // VerticalElevatorBindings.configureBindings(mechanismController, verticalElevator);

        // PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser("Auto-1");
        SmartDashboard.putData("Auto Mode", autoChooser);
        pathPlanner.configure();
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("Example Auto");
        return autoChooser.getSelected();
        // Load the path you want to follow using its name in the GUI
        // var path = PathPlannerPath.fromPathFile("Auto-1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        // return AutoBuilder.followPath(path);
    }
}
