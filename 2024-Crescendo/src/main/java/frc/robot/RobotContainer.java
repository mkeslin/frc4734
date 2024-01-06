package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

    // subsystems
    private Elevator horizontalElevator;
    private Elevator verticalElevator;

    final double MaxSpeed = 6; // 6 meters per second desired top speed
    final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

    // controllers
    CommandXboxController driveController = new CommandXboxController(XC1ID);
    CommandXboxController mechanismController = new CommandXboxController(XC2ID);

    // swerve drivetrain
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        // initialize subsystems
        horizontalElevator =
            new Elevator("horizontal", HORELEVATOR1ID, HORELEVATOR2ID, -200, -14000, -24000);
        verticalElevator =
            new Elevator("vertical", VERTELEVATOR1ID, VERTELEVATOR2ID, 3000, 45000, 45000);

        // configure bindings
        configureBindings_DriveTrain();
        configureBindings_HorizontalElevator();
        configureBindings_VerticalElevator();
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

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))
            );
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings_HorizontalElevator() {
        mechanismController
            .axisLessThan(CRY, -0.5)
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        horizontalElevator.movePositive();
                    },
                    horizontalElevator
                )
            );
        mechanismController
            .axisGreaterThan(CRY, 0.5)
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        horizontalElevator.moveNegative();
                    },
                    horizontalElevator
                )
            );

        // A Button: reset position
        mechanismController
            .a()
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        horizontalElevator.zero();
                    },
                    horizontalElevator
                )
            );
    }

    private void configureBindings_VerticalElevator() {
        mechanismController
            .axisLessThan(CRY, -0.5)
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        verticalElevator.movePositive();
                    },
                    verticalElevator
                )
            );
        mechanismController
            .axisGreaterThan(CRY, 0.5)
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        verticalElevator.moveNegative();
                    },
                    verticalElevator
                )
            );
        mechanismController
            .a()
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        verticalElevator.zero();
                    },
                    verticalElevator
                )
            );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
