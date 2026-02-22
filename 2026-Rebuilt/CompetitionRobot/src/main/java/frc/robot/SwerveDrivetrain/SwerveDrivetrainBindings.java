package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerBindingConstants;

/**
 * Configures controller bindings for the swerve drivetrain.
 * Supports three modes (Auto has no bindings; robot runs auto commands):
 * - TELEOP: Normal drive + teleop mechanism combos (intake/shoot/deploy).
 * - SYSID: Drivetrain and mechanism SysId characterization (PID tuning).
 * - MECHANISM: Tuning path + one button per mechanism (individual mechanism commands).
 */
public class SwerveDrivetrainBindings {

    /**
     * Input profile / mode for drive controller bindings.
     */
    public enum InputProfile {
        /** Teleop: normal driving; mechanism controller uses teleop commands. */
        TELEOP,
        /** SysId: drivetrain SysId; mechanism controller uses SysId commands. */
        SYSID,
        /** Mechanism: tuning path; mechanism controller uses individual mechanism commands. */
        MECHANISM
    }

    /**
     * Mechanism controller mode. Matches drive profile: TELEOP → TELEOP, SYSID → SYSID, MECHANISM → MECHANISM.
     */
    public enum MechanismMode {
        /** Teleop combo bindings (intake, shoot, deploy/stow). */
        TELEOP,
        /** SysId bindings (PID tuning). */
        SYSID,
        /** Individual mechanism buttons (one per mechanism for tuning). */
        MECHANISM
    }

    private static InputProfile currentProfile = InputProfile.TELEOP;
    private static MechanismMode currentMechanismMode = MechanismMode.TELEOP;

    private static double CurrentSpeed = DrivetrainConstants.MaxSpeed;
    private static double CurrentAngularRate = DrivetrainConstants.MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

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

    /**
     * Coordinate System Documentation:
     * 
     * Field-Centric Coordinate System:
     * - X+ axis: Away from driver station (toward opponent alliance wall)
     * - Y+ axis: Left (when facing away from driver station)
     * - Theta (rotation): Counter-clockwise is positive (0° = facing +X, 90° = facing +Y)
     * 
     * Coordinate Orientation:
     * - coordinateOrientation: Multiplier for field-centric coordinates
     *   - -1: Blue alliance (default) - standard field coordinates
     *   - 1: Red alliance - would flip coordinates if needed
     * - Currently static at -1 (blue alliance default)
     * - If alliance-based flipping is needed in the future, uncomment setAllianceOrientation()
     * 
     * Joystick Mapping:
     * - Left Stick Y+ (forward): Positive translation in field X direction
     * - Left Stick X+ (left): Positive translation in field Y direction
     * - Right Stick X+ (right): Positive rotation (counter-clockwise)
     * 
     * Note: The coordinateOrientation is kept static for simplicity. If your field is
     * rotationally symmetrical or you handle alliance flipping elsewhere (e.g., in PathPlanner),
     * keeping it static is appropriate.
     */
    private static int coordinateOrientation = -1;

    // public static void setAllianceOrientation(boolean isRed) {
    //     coordinateOrientation = isRed ? 1 : -1;
    // }

    /**
     * Applies scaled deadband to a joystick input value.
     * Scaled deadband provides smoother control near joystick center by scaling
     * the remaining range after deadband removal to [0, 1].
     * 
     * @param value The joystick input value (-1..1)
     * @param deadband The deadband threshold
     * @return The deadbanded and scaled value, or 0.0 if within deadband
     */
    private static double applyJoystickDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // Scale the remaining range to [0, 1] for smoother control
        return Math.signum(value) * ((Math.abs(value) - deadband) / (1.0 - deadband));
    }

    /**
     * Configures normal driving bindings.
     * This is the default profile for teleop operation.
     */
    private static void configureNormalBindings(CommandXboxController driveController, CommandSwerveDrivetrain drivetrain) {
        // Drivetrain will execute this command periodically

        // https://github.com/Operation-P-E-A-C-C-E-Robotics/frc-2025/blob/main/src/main/java/frc/robot/commands/drivetrain/PeaccyTuner.java

        // Sticks - only active in TELEOP profile
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    // Only drive if in TELEOP profile
                    if (currentProfile != InputProfile.TELEOP) {
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
                    
                    // Safety clamping to ensure velocities don't exceed maximums
                    // This provides a safety layer in case of calculation errors
                    velocityX = MathUtil.clamp(velocityX, -DrivetrainConstants.MaxSpeed, DrivetrainConstants.MaxSpeed);
                    velocityY = MathUtil.clamp(velocityY, -DrivetrainConstants.MaxSpeed, DrivetrainConstants.MaxSpeed);
                    rotationalRate = MathUtil.clamp(rotationalRate, -DrivetrainConstants.MaxAngularRate, DrivetrainConstants.MaxAngularRate);
                    
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

        // RIGHT BUMPER: Reset the field-centric heading (only in TELEOP profile)
        driveController.rightBumper()
                .and(() -> currentProfile == InputProfile.TELEOP)
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Turtle Mode while held
        driveController.leftBumper().onTrue(Commands.runOnce(() -> {
            CurrentSpeed = DrivetrainConstants.MaxSpeed * DrivetrainConstants.kTurtleSpeedMultiplier;
            CurrentAngularRate = DrivetrainConstants.kTurtleAngularRate;
        }));
        driveController.leftBumper().onFalse(Commands.runOnce(() -> {
            CurrentSpeed = DrivetrainConstants.MaxSpeed;
            CurrentAngularRate = DrivetrainConstants.MaxAngularRate;
        }));

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

        // Initial profile from constants; when switching is disabled this is the only profile
        setProfile(ControllerBindingConstants.DEFAULT_DRIVE_PROFILE);

        if (ControllerBindingConstants.ENABLE_PROFILE_SWITCHING) {
            // Profile switching: Hold Back + Start to cycle TELEOP → SYSID → MECHANISM → TELEOP
            driveController.back().and(driveController.start()).onTrue(
                    Commands.runOnce(() -> {
                        switch (currentProfile) {
                            case TELEOP -> currentProfile = InputProfile.SYSID;
                            case SYSID -> currentProfile = InputProfile.MECHANISM;
                            case MECHANISM -> currentProfile = InputProfile.TELEOP;
                        }
                        currentMechanismMode = profileToMechanismMode(currentProfile);
                    })
            );
        }
    }

    private static MechanismMode profileToMechanismMode(InputProfile profile) {
        return switch (profile) {
            case TELEOP -> MechanismMode.TELEOP;
            case SYSID -> MechanismMode.SYSID;
            case MECHANISM -> MechanismMode.MECHANISM;
        };
    }

    /**
     * Gets the current input profile (mode).
     *
     * @return The current profile (TELEOP, SYSID, or MECHANISM)
     */
    public static InputProfile getCurrentProfile() {
        return currentProfile;
    }

    /**
     * Sets the input profile. Also updates mechanism mode to match.
     *
     * @param profile The profile to set (TELEOP, SYSID, MECHANISM)
     */
    public static void setProfile(InputProfile profile) {
        currentProfile = profile;
        currentMechanismMode = profileToMechanismMode(profile);
    }

    /**
     * Gets the current mechanism controller mode (TELEOP, SYSID, or MECHANISM).
     *
     * @return The current mechanism mode
     */
    public static MechanismMode getMechanismMode() {
        return currentMechanismMode;
    }

    /**
     * Resets all rate limiters to their initial state.
     * Should be called when the robot is disabled to prevent stale state
     * from carrying over to the next enable cycle.
     */
    public static void resetRateLimiters() {
        RateLimiterX.reset(0.0);
        RateLimiterY.reset(0.0);
        RotationLimiter.reset(0.0);
    }
}
