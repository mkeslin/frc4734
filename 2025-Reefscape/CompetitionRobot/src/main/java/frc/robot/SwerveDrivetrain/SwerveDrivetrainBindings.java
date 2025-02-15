package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SwerveDrivetrainBindings {

    private static final double MaxSpeed = DrivetrainConstants.MaxSpeed;
    private static final double MaxAngularRate = DrivetrainConstants.MaxAngularRate;

    private static final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
    private static final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.
                                                                   // Adjust for max turning rate speed.

    private static double CurrentSpeed = MaxSpeed;
    private static double CurrentAngularRate = MaxAngularRate; // This will be updated when turtle and reset to
                                                               // MaxAngularRate

    private static SlewRateLimiter RateLimiterX = new SlewRateLimiter(0.5);
    private static SlewRateLimiter RateLimiterY = new SlewRateLimiter(0.5);

    // field-centric
    private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(CurrentSpeed * 0.1)
            .withRotationalDeadband(CurrentAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
    // private static final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    // private static final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();

    // robot-centric
    // private static final SwerveRequest.RobotCentric m_forwardStraight = new
    // SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private static final SwerveDrivetrainTelemetry m_logger = new SwerveDrivetrainTelemetry(MaxSpeed);

    // flip the orientation for blue/red
    private static int coordinateOrientation = -1;

    public static void setAllianceOrientation(boolean isRed) {
        coordinateOrientation = isRed ? 1 : -1;
    }

    public static void configureBindings(CommandXboxController driveController, CommandSwerveDrivetrain drivetrain) {
        // Drivetrain will execute this command periodically

        var velocityX = coordinateOrientation * driveController.getLeftY() * CurrentSpeed;
        var velocityXLimited = RateLimiterX.calculate(velocityX);

        var velocityY = coordinateOrientation * driveController.getLeftX() * CurrentSpeed;
        var velocityYLimited = RateLimiterY.calculate(velocityY);

        // Sticks
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> m_drive
                        .withVelocityX(velocityXLimited) // Drive forward with negative Y (forward)
                        .withVelocityY(velocityYLimited) // Drive left with negative X (left)
                        .withRotationalRate(-driveController.getRightX() * CurrentAngularRate) // Drive counterclockwise
                                                                                               // with negative X (left)
                ).ignoringDisable(false));

        // A Button: Brake
        // driveController.a().whileTrue(drivetrain.applyRequest(() -> m_brake));

        // B Button
        /*
         * driveController
         * .b()
         * .whileTrue(
         * drivetrain.applyRequest(() -> m_point.withModuleDirection(new Rotation2d(-driveController.getLeftY(),
         * -driveController.getLeftX())))
         * );
         */

        // LEFT BUMPER: Reset the field-centric heading
        // driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Turtle Mode while held
        driveController.leftBumper().onTrue(Commands.runOnce(() -> CurrentSpeed = MaxSpeed * TurtleSpeed)
                .andThen(() -> CurrentAngularRate = TurtleAngularRate));
        driveController.leftBumper().onFalse(
                Commands.runOnce(() -> CurrentSpeed = MaxSpeed).andThen(() -> CurrentAngularRate = MaxAngularRate));

        // test path
        // driveController
        // .leftBumper()
        // .whileTrue(
        // Commands.runOnce(
        // () -> {
        // // Load the path you want to follow using its name in the GUI
        // var path = PathPlannerPath.fromPathFile(pathFile);
        // // Create a path following command using AutoBuilder. This will also trigger event markers.
        // AutoBuilder.followPath(path);
        // },
        // drivetrain
        // )
        // );

        // if (Utils.isSimulation()) {
        // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        // }

        // TELEMETRY
        // drivetrain.registerTelemetry(m_logger::telemeterize);

        // SYSID
        // driveController.x().and(driveController.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
        // driveController.x().and(driveController.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

        // driveController.y().and(driveController.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
        // driveController.y().and(driveController.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

        // driveController.a().and(driveController.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
        // driveController.a().and(driveController.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

        // driveController.b().and(driveController.pov(0)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
        // driveController.b().and(driveController.pov(180)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));
    }
}
