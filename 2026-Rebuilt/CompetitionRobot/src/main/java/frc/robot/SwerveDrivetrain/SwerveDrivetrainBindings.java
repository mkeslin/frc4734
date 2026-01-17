package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Configures controller bindings for the swerve drivetrain.
 * Supports two input profiles:
 * - NORMAL: Standard driving and mechanism control
 * - SYSID: SysId characterization tests (disables normal driving)
 */
public class SwerveDrivetrainBindings {
    
    /**
     * Input profile mode for controller bindings.
     */
    public enum InputProfile {
        /** Normal driving and mechanism control */
        NORMAL,
        /** SysId characterization mode (disables normal driving) */
        SYSID
    }
    
    private static InputProfile currentProfile = InputProfile.NORMAL;

    private static final double MaxSpeed = DrivetrainConstants.MaxSpeed;
    private static final double MaxAngularRate = DrivetrainConstants.MaxAngularRate;

    private static double CurrentSpeed = MaxSpeed;
    private static double CurrentAngularRate = MaxAngularRate; // This will be updated when turtle and reset to
                                                               // MaxAngularRate

    // Rate limiters applied to normalized joystick inputs (-1..1) before scaling to physical units
    // These values are in 1/s (per second) and are applied to the joystick command inputs
    private static final SlewRateLimiter RateLimiterX = new SlewRateLimiter(DrivetrainConstants.kTranslationSlewRate);
    private static final SlewRateLimiter RateLimiterY = new SlewRateLimiter(DrivetrainConstants.kTranslationSlewRate);
    private static final SlewRateLimiter RotationLimiter = new SlewRateLimiter(DrivetrainConstants.kRotationSlewRate);

    // field-centric
    private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.0) // Deadband is now applied to joystick inputs, not physical units
            .withRotationalDeadband(0.0) // Deadband is now applied to joystick inputs, not physical units
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
    // private static final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    // private static final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();

    // robot-centric
    // private static final SwerveRequest.RobotCentric m_forwardStraight = new
    // SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private static final SwerveDrivetrainTelemetry m_logger = new SwerveDrivetrainTelemetry(MaxSpeed);

    // flip the orientation for blue/red
    private static int coordinateOrientation = -1;

    // public static void setAllianceOrientation(boolean isRed) {
    //     coordinateOrientation = isRed ? 1 : -1;
    // }

    /**
     * Applies deadband to a joystick input value.
     * 
     * @param value The joystick input value (-1..1)
     * @param deadband The deadband threshold
     * @return The deadbanded value, or 0.0 if within deadband
     */
    private static double applyJoystickDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    /**
     * Configures normal driving bindings.
     * This is the default profile for teleop operation.
     */
    private static void configureNormalBindings(CommandXboxController driveController, CommandSwerveDrivetrain drivetrain) {
        // Drivetrain will execute this command periodically

        // https://github.com/Operation-P-E-A-C-C-E-Robotics/frc-2025/blob/main/src/main/java/frc/robot/commands/drivetrain/PeaccyTuner.java

        // Sticks - only active in NORMAL profile
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    // Only drive if in NORMAL profile
                    if (currentProfile != InputProfile.NORMAL) {
                        return new SwerveRequest.SwerveDriveBrake();
                    }
                    
                    // Read raw joystick inputs
                    var rawX = driveController.getLeftY();
                    var rawY = driveController.getLeftX();
                    var rawRotation = -driveController.getRightX();
                    
                    // Apply deadband to joystick inputs (preferred order: deadband first)
                    var xWithDeadband = applyJoystickDeadband(rawX, DrivetrainConstants.kJoystickDeadband);
                    var yWithDeadband = applyJoystickDeadband(rawY, DrivetrainConstants.kJoystickDeadband);
                    var rotationWithDeadband = applyJoystickDeadband(rawRotation, DrivetrainConstants.kJoystickDeadband);
                    
                    // Apply rate limiting to normalized joystick inputs (-1..1)
                    // This smooths out sudden joystick movements before scaling to physical units
                    var xLimited = RateLimiterX.calculate(xWithDeadband);
                    var yLimited = RateLimiterY.calculate(yWithDeadband);
                    var rotationLimited = RotationLimiter.calculate(rotationWithDeadband);
                    
                    // Scale rate-limited joystick inputs to physical units (m/s and rad/s)
                    var velocityX = coordinateOrientation * xLimited * CurrentSpeed;
                    var velocityY = coordinateOrientation * yLimited * CurrentSpeed;
                    var rotationalRate = rotationLimited * CurrentAngularRate;
                    
                    // Apply rate-limited velocities to drive command
                    return m_drive
                            .withVelocityX(velocityX) // Drive forward with negative Y (forward)
                            .withVelocityY(velocityY) // Drive left with negative X (left)
                            .withRotationalRate(rotationalRate); // Drive counterclockwise with negative X (left)
                }).ignoringDisable(false));

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

        // RIGHT BUMPER: Reset the field-centric heading (only in NORMAL profile)
        driveController.rightBumper()
                .and(() -> currentProfile == InputProfile.NORMAL)
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Turtle Mode while held
        driveController.leftBumper().onTrue(Commands.runOnce(() -> CurrentSpeed = MaxSpeed * DrivetrainConstants.kTurtleSpeedMultiplier)
                .andThen(() -> CurrentAngularRate = DrivetrainConstants.kTurtleAngularRate));
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

    }

    /**
     * Configures SysId characterization bindings.
     * These bindings are only active when in SYSID profile mode.
     * 
     * SysId Bindings (when in SYSID profile):
     * - Back + Y: Dynamic Forward
     * - Back + X: Dynamic Reverse
     * - Start + Y: Quasistatic Forward
     * - Start + X: Quasistatic Reverse
     * - A Button (alone): Switch to Translation SysId
     * - B Button (alone): Switch to Steer SysId
     * - Right Bumper (alone): Switch to Rotation SysId
     */
    private static void configureSysIdBindings(CommandXboxController driveController, CommandSwerveDrivetrain drivetrain) {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // These commands only work when in SYSID profile
        driveController.back().and(driveController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward)
                        .onlyIf(() -> currentProfile == InputProfile.SYSID));
        driveController.back().and(driveController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)
                        .onlyIf(() -> currentProfile == InputProfile.SYSID));
        driveController.start().and(driveController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)
                        .onlyIf(() -> currentProfile == InputProfile.SYSID));
        driveController.start().and(driveController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)
                        .onlyIf(() -> currentProfile == InputProfile.SYSID));
        
        // SysId routine selection (only active in SYSID profile, and only when buttons are pressed alone)
        // Use .and() with negated conditions to ensure buttons aren't being used for other purposes
        driveController.a()
                .and(() -> !driveController.back().getAsBoolean())
                .and(() -> !driveController.start().getAsBoolean())
                .onTrue(Commands.runOnce(() -> {
                    if (currentProfile == InputProfile.SYSID) {
                        drivetrain.setSysIdRoutine(CommandSwerveDrivetrain.SysIdRoutineType.TRANSLATION);
                    }
                }));
        driveController.b()
                .and(() -> !driveController.back().getAsBoolean())
                .and(() -> !driveController.start().getAsBoolean())
                .onTrue(Commands.runOnce(() -> {
                    if (currentProfile == InputProfile.SYSID) {
                        drivetrain.setSysIdRoutine(CommandSwerveDrivetrain.SysIdRoutineType.STEER);
                    }
                }));
        // Use right bumper for rotation (left bumper is used for turtle mode in normal profile)
        driveController.rightBumper()
                .and(() -> !driveController.back().getAsBoolean())
                .and(() -> !driveController.start().getAsBoolean())
                .onTrue(Commands.runOnce(() -> {
                    if (currentProfile == InputProfile.SYSID) {
                        drivetrain.setSysIdRoutine(CommandSwerveDrivetrain.SysIdRoutineType.ROTATION);
                    }
                }));
    }

    /**
     * Configures all bindings for the drivetrain.
     * Sets up both normal driving and SysId bindings.
     * 
     * @param driveController The drive controller
     * @param drivetrain The swerve drivetrain
     */
    public static void configureBindings(CommandXboxController driveController, CommandSwerveDrivetrain drivetrain) {
        configureNormalBindings(driveController, drivetrain);
        configureSysIdBindings(driveController, drivetrain);
        
        // Profile switching: Hold Back + Start to toggle between NORMAL and SYSID modes
        driveController.back().and(driveController.start()).onTrue(
                Commands.runOnce(() -> {
                    currentProfile = (currentProfile == InputProfile.NORMAL) 
                            ? InputProfile.SYSID 
                            : InputProfile.NORMAL;
                })
        );
    }

    /**
     * Gets the current input profile.
     * 
     * @return The current input profile (NORMAL or SYSID)
     */
    public static InputProfile getCurrentProfile() {
        return currentProfile;
    }

    /**
     * Sets the input profile.
     * 
     * @param profile The profile to set (NORMAL or SYSID)
     */
    public static void setProfile(InputProfile profile) {
        currentProfile = profile;
    }
}
