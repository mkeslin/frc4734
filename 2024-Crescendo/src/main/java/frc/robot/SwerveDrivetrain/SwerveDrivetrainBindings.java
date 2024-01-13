package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDrivetrainBindings {

    private static final double MaxSpeed = SwerveDrivetrainConstants.MaxSpeed;
    private static final double MaxAngularRate = SwerveDrivetrainConstants.MaxAngularRate;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // field-centric driving in open loop
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private static final SwerveDrivetrainTelemetry logger = new SwerveDrivetrainTelemetry(MaxSpeed);

    public static void configureBindings(
        CommandXboxController driveController,
        CommandSwerveDrivetrain drivetrain
    ) {
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
        // driveController
        //     .leftBumper()
        //     .whileTrue(
        //         Commands.runOnce(
        //             () -> {
        //                 // Load the path you want to follow using its name in the GUI
        //                 var path = PathPlannerPath.fromPathFile(pathFile);
        //                 // Create a path following command using AutoBuilder. This will also trigger event markers.
        //                 AutoBuilder.followPath(path);
        //             },
        //             drivetrain
        //         )
        //     );

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))
            );
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
