package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDrivetrainBindings {

    private static final double MaxSpeed = DrivetrainConstants.MaxSpeed;
    private static final double MaxAngularRate = DrivetrainConstants.MaxAngularRate;

    // field-centric
    private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
        // .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private static final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
    
    // robot-centric
    // private static final SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // private static final SwerveDrivetrainTelemetry m_logger = new SwerveDrivetrainTelemetry(MaxSpeed);
    
    // flip the orientation for blue/red
    private static int coordinateOrientation = -1;
    public static void setAllianceOrientation(boolean isRed) {
        coordinateOrientation = isRed ? 1 : -1;
    }

    public static void configureBindings(CommandXboxController driveController, CommandSwerveDrivetrain drivetrain) {
        // Drivetrain will execute this command periodically
        // Sticks
        

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                m_drive
                    .withVelocityX(coordinateOrientation * driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(coordinateOrientation * driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)

            ).ignoringDisable(false)
        );

        // A Button: Brake
        driveController.a().whileTrue(drivetrain.applyRequest(() -> m_brake));

        // B Button
        /*driveController
            .b()
            .whileTrue(
                drivetrain.applyRequest(() -> m_point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX())))
            );*/

        // LEFT BUMPER: Reset the field-centric heading
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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

        // if (Utils.isSimulation()) {
        //     drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        // }

        // drivetrain.registerTelemetry(m_logger::telemeterize);

        // driveController.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        // driveController.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        /* Bindings for drivetrain characterization */
        /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
        /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
        // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }
}
