package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                // new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                DrivetrainConstants.MaxSpeed, // Max module speed, in m/s
                Units.inchesToMeters(15.026), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here,
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
    // private static final double MaxSpeed = DrivetrainConstants.MaxSpeed;
    // private static final double MaxAngularRate = DrivetrainConstants.MaxAngularRate;

    // private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    //     .withDeadband(MaxSpeed * 0.1)
    //     .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // field-centric driving in open loop
    //     .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        // .withDeadband(MaxSpeed * 0.1)
        // .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity) // robot-centric driving based on velocity
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public Pose2d getPose() {
        // return this.m_cachedState.Pose;
        return this.m_odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        this.m_odometry.resetPosition(pose.getRotation(), this.m_modulePositions, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // return this.m_kinematics.toChassisSpeeds(this.m_cachedState.ModuleStates);

        // var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

        var vx = driveRequest.VelocityX;
        var vy = driveRequest.VelocityY;
        var omega = driveRequest.RotationalRate;
        return new ChassisSpeeds(vx, vy, omega);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        this.setControl(
                driveRequest
                    .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                    .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                    .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
            );
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
