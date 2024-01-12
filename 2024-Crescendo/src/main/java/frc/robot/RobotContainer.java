package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerIds;
import frc.robot.SwerveDrivetrain.*;
import java.util.List;

public class RobotContainer {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ROBOT SELECTOR
    private final CommandSwerveDrivetrain selectedDrivetrain = DrivetrainA.DriveTrain;
    // private final CommandSwerveDrivetrain selectedDrivetrain = DrivetrainB.DriveTrain;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // private String pathFile = Filesystem.getDeployDirectory().getPath() + "/pathplanner/paths/Auto-1.path";
    private String pathFile = "Auto-1";

    // subsystems
    // private Elevator horizontalElevator;
    // private Elevator verticalElevator;

    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    // controllers
    CommandXboxController driveController = new CommandXboxController(ControllerIds.XC1ID);
    //   CommandXboxController mechanismController = new CommandXboxController(ControllerIds.XC2ID);

    // swerve drivetrain
    private final CommandSwerveDrivetrain drivetrain = selectedDrivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // field-centric driving in open loop
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    Telemetry logger = new Telemetry(MaxSpeed);

    // PathPlanner
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // initialize subsystems
        // horizontalElevator =
        //     new Elevator("horizontal", HORELEVATOR1ID, HORELEVATOR2ID, -200, -14000, -24000);
        // verticalElevator =
        //     new Elevator("vertical", VERTELEVATOR1ID, VERTELEVATOR2ID, 3000, 45000, 45000);

        // Register Named Commands
        // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
        // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
        // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

        // configure bindings
        configureBindings_DriveTrain();
        configureBindings_PathPlanner();
        configureBindings_HorizontalElevator();
        configureBindings_VerticalElevator();

        // PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings_DriveTrain() {
        // Drivetrain will execute this command periodically
        // Sticks
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // A Button: Brake
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B Button
        driveController
            .b()
            .whileTrue(
                drivetrain.applyRequest(() ->
                    point.withModuleDirection(
                        new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX())
                    )
                )
            );

        // test path
        driveController
            .leftBumper()
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        // Load the path you want to follow using its name in the GUI
                        var path = PathPlannerPath.fromPathFile(pathFile);
                        // Create a path following command using AutoBuilder. This will also trigger event markers.
                        AutoBuilder.followPath(path);
                    },
                    drivetrain
                )
            );

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))
            );
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings_PathPlanner() {
        // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

        // Add a button to run pathfinding commands to SmartDashboard
        SmartDashboard.putData(
            "Pathfind to closest notes",
            // AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathFile))
            AutoBuilder.pathfindToPose(
                new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
                new PathConstraints(
                    4.0,
                    4.0,
                    Units.degreesToRadians(360),
                    Units.degreesToRadians(540)
                ),
                0,
                2.0
            )
        );
        SmartDashboard.putData(
            "Pathfind to not implemented",
            // AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathFile))
            AutoBuilder.pathfindToPose(
                new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
                new PathConstraints(
                    4.0,
                    4.0,
                    Units.degreesToRadians(360),
                    Units.degreesToRadians(540)
                ),
                0,
                0
            )
        );

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m in the +X field direction
        SmartDashboard.putData(
            "On-the-fly path",
            Commands.runOnce(() -> {
                Pose2d currentPose = drivetrain.getPose();

                // The rotation component in these poses represents the direction of travel
                Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
                Pose2d endPos = new Pose2d(
                    currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
                    new Rotation2d()
                );

                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    startPos,
                    endPos
                );
                PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(
                        4.0,
                        4.0,
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(540)
                    ),
                    new GoalEndState(0.0, currentPose.getRotation())
                );

                // Prevent this path from being flipped on the red alliance, since the given positions are already correct
                path.preventFlipping = true;

                AutoBuilder.followPath(path).schedule();
            })
        );
    }

    private void configureBindings_HorizontalElevator() {
        // mechanismController
        //     .axisLessThan(CRY, -0.5)
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 horizontalElevator.movePositive();
        //             },
        //             horizontalElevator
        //         )
        //     );
        // mechanismController
        //     .axisGreaterThan(CRY, 0.5)
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 horizontalElevator.moveNegative();
        //             },
        //             horizontalElevator
        //         )
        //     );

        // // A Button: reset position
        // mechanismController
        //     .a()
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 horizontalElevator.zero();
        //             },
        //             horizontalElevator
        //         )
        //     );
    }

    private void configureBindings_VerticalElevator() {
        // mechanismController
        //     .axisLessThan(CRY, -0.5)
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 verticalElevator.movePositive();
        //             },
        //             verticalElevator
        //         )
        //     );
        // mechanismController
        //     .axisGreaterThan(CRY, 0.5)
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 verticalElevator.moveNegative();
        //             },
        //             verticalElevator
        //         )
        //     );
        // mechanismController
        //     .a()
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 verticalElevator.zero();
        //             },
        //             verticalElevator
        //         )
        //     );
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
