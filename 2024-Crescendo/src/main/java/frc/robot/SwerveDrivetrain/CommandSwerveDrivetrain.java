package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants,
        double OdometryUpdateFrequency,
        SwerveModuleConstants... modules
    ) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants,
        SwerveModuleConstants... modules
    ) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        // PathPlanner
        AutoBuilder.configureHolonomic(
            // Robot pose supplier
            this::getPose2,
            // Method to reset odometry (will be called if your auto has a starting pose)
            this::resetPose2,
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::getRobotRelativeSpeeds2,
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            this::driveRobotRelative2,
            // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(), // Default path replanning config. See the API for the options here,
                0.2
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
        // Set up custom logging to add the current path to a field 2d widget
        // PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
            new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - m_lastSimTime;
                m_lastSimTime = currentTime;

                /* use the measured time delta, get battery voltage from WPILib */
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // For use with PathPlanner
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    public Pose2d getPose2() {
        return this.m_odometry.getEstimatedPosition();
    }

    public void resetPose2(Pose2d pose) {
        this.m_odometry.resetPosition(pose.getRotation(), this.m_modulePositions, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds2() {
        return this.m_kinematics.toChassisSpeeds(this.m_cachedState.ModuleStates);
    }

    public void driveRobotRelative2(ChassisSpeeds chassisSpeeds) {
        // this.driveRobotRelative(chassisSpeeds);

        // chassisSpeeds.
        // this.m_kinematics.toChassisSpeeds(chassisSpeeds)

        this.applyRequest(() ->
                drive
                    .withVelocityX(chassisSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                    .withVelocityY(chassisSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
                    .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond) // Drive counterclockwise with negative X (left)
            );
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
