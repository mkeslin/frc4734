package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainA.TunerSwerveDrivetrain;

import java.util.function.Supplier;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // private static final double m_simLoopPeriod = 0.005; // 5 ms
    // private Notifier m_simNotifier = null;
    // private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d m_blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d m_redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // private final SwerveRequest.ApplyChassisSpeeds m_autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // private final SwerveRequest.SysIdSwerveTranslation
    // TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
    // new SwerveRequest.SysIdSwerveRotation();
    // private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new
    // SwerveRequest.SysIdSwerveSteerGains();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        // configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        // configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        // configureAutoBuilder();
    }

    // private void configureAutoBuilder() {
    //     double driveBaseRadius = 0;
    //     for (var moduleLocation : m_moduleLocations) {
    //         driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    //     }

    //     AutoBuilder.configure(
    //             this::getPose, // Robot pose supplier
    //             this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //             new HolonomicPathFollowerConfig(
    //                     new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
    //                     new PIDConstants(10.0, 0.0, 0.0), // Rotation PID constants
    //                     DrivetrainConstants.MaxSpeed, // Max module speed, in m/s
    //                     // Units.inchesToMeters(15.026), // Drive base radius in meters. Distance from
    //                     // robot center to furthest module.
    //                     driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //                     new ReplanningConfig() // Default path replanning config. See the API for the options here,
    //             ),
    //             () -> {
    //                 // Boolean supplier that controls when the path will be mirrored for the red
    //                 // alliance
    //                 // This will flip the path being followed to the red side of the field.
    //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                     return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //             },
    //             this // Reference to this subsystem to set requirements
    //     );
    //     // Set up custom logging to add the current path to a field 2d widget
    //     // PathPlannerLogging.setLogActivePathCallback(poses ->
    //     // field.getObject("path").setPoses(poses));
    // }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // return RoutineToApply.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // return RoutineToApply.dynamic(direction);
    // }

    // private void startSimThread() {
    // m_lastSimTime = Utils.getCurrentTimeSeconds();

    // /* Run simulation at a faster rate so PID gains behave more reasonably */
    // m_simNotifier =
    // new Notifier(() -> {
    // final double currentTime = Utils.getCurrentTimeSeconds();
    // double deltaTime = currentTime - m_lastSimTime;
    // m_lastSimTime = currentTime;

    // /* use the measured time delta, get battery voltage from WPILib */
    // updateSimState(deltaTime, RobotController.getBatteryVoltage());
    // });
    // m_simNotifier.startPeriodic(m_simLoopPeriod);
    // }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // For use with PathPlanner

    // public Pose2d getPose() {
    //     return this.getState().Pose;
    // }

    // public void resetPose(Pose2d pose) {
    //     this.seedFieldRelative(pose);
    // }

    // public ChassisSpeeds getRobotRelativeSpeeds() {
    //     return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    // }

    // public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    //     this.setControl(m_autoRequest.withSpeeds(chassisSpeeds));
    // }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation
                    .getAlliance()
                    .ifPresent(allianceColor -> {
                        this.setOperatorPerspectiveForward(
                                allianceColor == Alliance.Red ? m_redAlliancePerspectiveRotation
                                        : m_blueAlliancePerspectiveRotation);
                        m_hasAppliedOperatorPerspective = true;
                    });
        }
    }
}
