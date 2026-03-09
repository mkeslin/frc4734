package frc.robot.Auto.commands;

import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Logging.RobotLogger;
import frc.robot.PathPlanner.AllianceUtils;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Builder class for creating autonomous routines from atomic commands.
 * 
 * <p>
 * This class provides static factory methods that compose atomic commands
 * into complete autonomous routines. It demonstrates command composition
 * patterns including parallel execution, sequencing, and conditional logic.
 * 
 * <p>
 * Example usage:
 * 
 * <pre>
 * Command climberAuto = AutoRoutines.buildClimberAuto(
 *         StartPoseId.POS_1,
 *         "PathToShot",
 *         "PathToTower",
 *         towerAlignPose,
 *         180.0);
 * </pre>
 */
public class AutoRoutines {
    private AutoRoutines() {
        // Utility class - prevent instantiation
    }

    /**
     * Builds a climber autonomous routine using drive-to poses (no PathPlanner
     * paths).
     * 
     * <p>
     * This routine:
     * <ol>
     * <li>Seeds odometry from start pose</li>
     * <li>Applies tag snap if vision quality is good</li>
     * <li>Drives to shot position (midpoint between start and tower align;
     * parallel: spins up shooter)</li>
     * <li>Lowers intake if present (deploy so webcam is not blocked)</li>
     * <li>Acquires hub aim</li>
     * <li>Waits for shooter at speed</li>
     * <li>Shoots preload for specified duration</li>
     * <li>Drives to tower align pose (offset 2 ft toward field center from
     * bar)</li>
     * <li>Applies tag snap before final alignment</li>
     * <li>Drives to tower align pose again (fine alignment)</li>
     * <li>Extends climber to L1 at offset pose</li>
     * <li>Drives 2 ft toward the bar (field-relative) to acquire the bar</li>
     * <li>Retracts climber from L1</li>
     * <li>Holds climb until auto end</li>
     * </ol>
     * 
     * @param id                 Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers (evaluated at
     *                           runtime for correct alliance)
     * @param shotPoseSupplier   Supplier of target pose for shooting (e.g. midpoint
     *                           of start and tower align)
     * @param towerAlignPose     Supplier of target pose for tower/climb (e.g. from
     *                           Shuffleboard climb-side chooser)
     * @param fallbackHeadingDeg Fallback heading for hub aiming (degrees)
     * @param targetRpmSupplier  Supplier of target shooter RPM (e.g. from Preferences for runtime tuning)
     * @param rpmTol             RPM tolerance for shooter at-speed check
     * @param shootDurationSupplier Supplier of shoot duration (seconds); read at runtime for tuning
     * @param drivetrain         The drivetrain subsystem
     * @param vision             The PhotonVision subsystem
     * @param shooter            The shooter subsystem
     * @param feeder             The feeder subsystem
     * @param intake             Optional deployable intake; if non-null, intake is
     *                           lowered before hub aim so webcam is unblocked
     * @param climber            The climber subsystem
     * @return A command representing the complete climber auto routine
     * @throws NullPointerException if any required parameter is null
     */
    public static Command buildClimberAuto(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            Supplier<Pose2d> shotPoseSupplier,
            Supplier<Pose2d> towerAlignPose,
            double fallbackHeadingDeg,
            Supplier<Double> targetRpmSupplier,
            double rpmTol,
            Supplier<Double> shootDurationSupplier,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake,
            Climber climber) {

        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(shotPoseSupplier, "shotPoseSupplier cannot be null");
        Objects.requireNonNull(towerAlignPose, "towerAlignPose cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        Objects.requireNonNull(climber, "climber cannot be null");
        Objects.requireNonNull(targetRpmSupplier, "targetRpmSupplier cannot be null");
        Objects.requireNonNull(shootDurationSupplier, "shootDurationSupplier cannot be null");

        Supplier<Double> rpmSupplier = targetRpmSupplier;

        // Captured after extending climber; used by drive-to-bar step (pathfind to pose
        // toward bar).
        final Pose2d[] poseBeforeDriveToBar = new Pose2d[1];
        return Commands.sequence(
                // ----- Step 1: Seed odometry -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 1: Seed odometry")),
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // ----- Step 2: Tag snap -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 2: Tag snap")),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 3: Drive to shot pose -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 3: Drive to shot")),
                Commands.deadline(
                        CmdDriveToPose.create(
                                drivetrain,
                                shotPoseSupplier,
                                AutoConstants.DEFAULT_XY_TOLERANCE,
                                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                                AutoConstants.DEFAULT_PATH_TIMEOUT),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // ----- Step 4: Lower intake -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 4: Lower intake")),
                intake != null
                        ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                                .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                        : Commands.none(),

                // ----- Step 5: Acquire hub aim -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 5: Acquire hub aim")),
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // ----- Step 6: Wait shooter at speed -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 6: Wait shooter at speed")),
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),

                // ----- Step 7: Shoot preload -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 7: Shoot")),
                Commands.deferredProxy(() -> CmdShootForTime.create(shooter, feeder, floor, shootDurationSupplier.get())),

                // ----- Step 8: Drive to tower align -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 8: Drive to tower")),
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // ----- Step 9: Tag snap before final alignment -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 9: Tag snap")),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 10: Fine alignment to tower -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 10: Fine alignment")),
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // 11. Extend climber to L1 at offset pose (no retract yet)
                ClimbWhileHeldCommand.extendL1Only(climber, drivetrain)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // ----- Step 12: Drive toward bar to acquire -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 12: Drive toward bar")),
                Commands.runOnce(() -> poseBeforeDriveToBar[0] = drivetrain.getPose()),
                CmdDriveToPose.create(
                        drivetrain,
                        () -> {
                            Pose2d p = poseBeforeDriveToBar[0];
                            if (p == null)
                                return null;
                            // Target is toward bar (in blue: -X). Total = main drive +
                            // extra.
                            double totalDriveMeters = AutoConstants.CLIMB_DRIVE_TO_BAR_METERS
                                    + AutoConstants.CLIMB_EXTRA_DRIVE_TOWARD_BAR_METERS;
                            Pose2d pBlue = DriverStation.getAlliance()
                                    .orElse(Alliance.Blue) == Alliance.Red
                                            ? AllianceUtils.redToBlue(p)
                                            : p;
                            return new Pose2d(
                                    pBlue.getTranslation().minus(new Translation2d(
                                            totalDriveMeters, 0)),
                                    pBlue.getRotation());
                        },
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // ----- Step 13: Nudge toward bar -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 13: Nudge toward bar")),
                CmdDriveFieldRelative.forDistance(
                        drivetrain,
                        -AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        0.25),

                // ----- Step 14: Retract climber from L1 -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 14: Retract climber")),
                ClimbWhileHeldCommand.retractFromL1(climber, drivetrain)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // ----- Step 15: Hold climb until end -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 15: Hold climb")),
                new CmdHoldClimbUntilEnd(climber, drivetrain),

                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Complete")))
                .withName("ClimberAuto");
    }

    /**
     * Builds a shooter autonomous routine.
     * 
     * <p>
     * This routine:
     * <ol>
     * <li>Seeds odometry from start pose</li>
     * <li>Applies tag snap if vision quality is good</li>
     * <li>Follows path to shot position (parallel: spins up shooter)</li>
     * <li>Lowers intake</li>
     * <li>Acquires hub aim</li>
     * <li>Waits for shooter at speed</li>
     * <li>Shoots for specified duration</li>
     * <li>Follows path to center (parallel: intake on until path endpoint)</li>
     * <li>Follows path back to shot (parallel: spin up shooter)</li>
     * <li>Acquires hub aim</li>
     * <li>Waits for shooter at speed</li>
     * <li>Shoots for specified duration</li>
     * </ol>
     * 
     * @param id                 Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers (evaluated at
     *                           runtime for correct alliance)
     * @param pathToShot         Path name to shooting position
     * @param pathToCenter       Path name to center
     * @param pathBackToShot     Path name back to shot
     * @param fallbackHeadingDeg Fallback heading for hub aiming (degrees)
     * @param targetRpmSupplier  Supplier of target shooter RPM (e.g. from Preferences for runtime tuning)
     * @param rpmTol             RPM tolerance for shooter at-speed check
     * @param shootDurationSupplier Supplier of shoot duration (seconds); read at runtime for tuning
     * @param drivetrain         The drivetrain subsystem
     * @param vision             The PhotonVision subsystem
     * @param shooter            The shooter subsystem
     * @param feeder             The feeder subsystem
     * @param floor              The floor conveyor subsystem (runs with feeder to
     *                           feed notes)
     * @param intake             The intake subsystem
     * @return A command representing the complete shooter auto routine
     * @throws NullPointerException if any required parameter is null
     */
    public static Command buildShooterAuto(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathToShot,
            String pathToCenter,
            String pathBackToShot,
            double fallbackHeadingDeg,
            Supplier<Double> targetRpmSupplier,
            double rpmTol,
            Supplier<Double> shootDurationSupplier,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake) {

        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(pathToShot, "pathToShot cannot be null");
        Objects.requireNonNull(pathToCenter, "pathToCenter cannot be null");
        Objects.requireNonNull(pathBackToShot, "pathBackToShot cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        Objects.requireNonNull(floor, "floor cannot be null");
        Objects.requireNonNull(intake, "intake cannot be null");
        Objects.requireNonNull(targetRpmSupplier, "targetRpmSupplier cannot be null");
        Objects.requireNonNull(shootDurationSupplier, "shootDurationSupplier cannot be null");

        Supplier<Double> rpmSupplier = targetRpmSupplier;

        return Commands.sequence(
                // ----- Step 1: Seed odometry -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 1: Seed odometry")),
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // ----- Step 2: Lower intake (webcam unblocked for hub aim) -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 2: Lower intake")),
                intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                        .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT),

                // ----- Step 3: Tag snap -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 3: Tag snap")),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 4: Spin up shooter (1.5s spin, then wait for at speed or 5s timeout) -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 4: Spin up shooter")),
                Commands.sequence(
                        Commands.deadline(
                                Commands.waitSeconds(AutoConstants.SHOOTER_SPINUP_DURATION_LEFT_RIGHT),
                                new CmdShooterSpinUp(shooter, rpmSupplier)),
                        new CmdWaitShooterAtSpeed(shooter, rpmSupplier, rpmTol, 5.0)),

                // Step 5 (Acquire hub aim) commented out in current config.

                // ----- Step 6: Shoot -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 6: Shoot")),
                Commands.deferredProxy(() -> CmdShootForTime.create(shooter, feeder, floor, shootDurationSupplier.get(), AutoConstants.SHOOT_SPINUP_DELAY_LEFT_RIGHT, rpmSupplier)),

                // ----- TEMPORARY: Path to center and second shot commented out -----
                // Uncomment to restore full routine: path to center, path back, acquire aim,
                // shoot again.
                // Commands.deadline(
                // new CmdFollowPath(pathToCenter, AutoConstants.DEFAULT_PATH_TIMEOUT,
                // drivetrain),
                // CmdIntakeOn.create(intake)),
                // Commands.deadline(
                // new CmdFollowPath(pathBackToShot, AutoConstants.DEFAULT_PATH_TIMEOUT,
                // drivetrain),
                // new CmdShooterSpinUp(shooter, rpmSupplier)),
                // CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),
                // CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),
                // CmdShootForTime.create(shooter, feeder, floor, shootDuration),

                // ----- Step 7: Raise intake (reset for next run) -----
                // Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 7: Raise intake")),
                // intake.moveToSetDeployPositionCommand(() -> DeployPosition.STOWED)
                //         .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT),

                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Complete")))
                .withName("ShooterAuto");
    }

    /**
     * Builds a test routine: seed, drive to shot, lower intake, aim, shoot (no
     * center run).
     * Subset of ShooterAuto for testing drive + shoot flow.
     *
     * @param id                 Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers
     * @param pathToShot         Path name to shooting position
     * @param fallbackHeadingDeg Fallback heading for hub aiming
     * @param targetRpmSupplier  Supplier of target shooter RPM (e.g. from Preferences for runtime tuning)
     * @param rpmTol             RPM tolerance
     * @param shootDurationSupplier Supplier of shoot duration (seconds); read at runtime for tuning
     * @param drivetrain         Drivetrain subsystem
     * @param vision             Vision subsystem
     * @param shooter            Shooter subsystem
     * @param feeder             Feeder subsystem
     * @param floor              Optional floor subsystem; if non-null, runs after
     *                           delay to feed from conveyor
     * @param intake             Optional deployable intake; if non-null, lowered
     *                           before hub aim so webcam is unblocked
     */
    public static Command buildTestDriveAndShoot(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathToShot,
            double fallbackHeadingDeg,
            Supplier<Double> targetRpmSupplier,
            double rpmTol,
            Supplier<Double> shootDurationSupplier,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake) {
        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(pathToShot, "pathToShot cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        Objects.requireNonNull(targetRpmSupplier, "targetRpmSupplier cannot be null");
        Objects.requireNonNull(shootDurationSupplier, "shootDurationSupplier cannot be null");

        Supplier<Double> rpmSupplier = targetRpmSupplier;

        return Commands.sequence(
                // ----- Step 1: Seed odometry -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 1: Seed odometry")),
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // ----- Step 2: Tag snap -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 2: Tag snap")),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 3: Drive to shoot spot + spin up shooter (drive is deadline; spin-up runs during drive) -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 3: Drive to shot + spin up shooter")),
                Commands.deadline(
                        new CmdFollowPath(pathToShot, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // ----- Step 4: Lower intake -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 4: Lower intake")),
                intake != null
                        ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                                .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                        : Commands.none(),

                // ----- Step 5: Acquire hub aim -----
                // Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 5: Acquire hub aim")),
                // CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // ----- Step 6: Shoot -----
                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 6: Shoot")),
                Commands.deferredProxy(() -> CmdShootForTime.create(shooter, feeder, floor, shootDurationSupplier.get())),

                // ----- Step 7: Raise intake (reset for next run) -----
                // Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 7: Raise intake")),
                // intake != null
                //         ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.STOWED)
                //                 .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                //         : Commands.none(),

                Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Complete"))
        )
        .withName("TestDriveAndShoot");
    }

    /**
     * Builds a test routine: seed, drive to tower, extend L1, drive to bar,
     * retract.
     * Subset of ClimberAuto for testing climb flow.
     *
     * @param id                 Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers
     * @param towerAlignPose     Supplier of tower align pose (from climb-side
     *                           chooser)
     * @param drivetrain         Drivetrain subsystem
     * @param vision             Vision subsystem
     * @param climber            Climber subsystem
     */
    public static Command buildTestClimb(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            Supplier<Pose2d> towerAlignPose,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Climber climber) {
        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(towerAlignPose, "towerAlignPose cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(climber, "climber cannot be null");

        final Pose2d[] poseBeforeDriveToBar = new Pose2d[1];

        return Commands.sequence(
                // ----- Step 1: Seed odometry -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 1: Seed odometry")),
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // ----- Step 2: Tag snap -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 2: Tag snap")),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 3: Drive to tower align -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 3: Drive to tower")),
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // ----- Step 4: Tag snap again -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 4: Tag snap")),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 5: Fine alignment to tower -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 5: Fine alignment")),
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // ----- Step 6: Extend climber to L1 -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 6: Extend climber L1")),
                ClimbWhileHeldCommand.extendL1Only(climber, drivetrain)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // ----- Step 7: Drive toward bar to acquire -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 7: Drive toward bar")),
                Commands.runOnce(() -> poseBeforeDriveToBar[0] = drivetrain.getPose()),
                CmdDriveToPose.create(
                        drivetrain,
                        () -> {
                            Pose2d p = poseBeforeDriveToBar[0];
                            if (p == null)
                                return null;
                            double totalDriveMeters = AutoConstants.CLIMB_DRIVE_TO_BAR_METERS
                                    + AutoConstants.CLIMB_EXTRA_DRIVE_TOWARD_BAR_METERS;
                            Pose2d pBlue = DriverStation.getAlliance()
                                    .orElse(Alliance.Blue) == Alliance.Red
                                            ? AllianceUtils.redToBlue(p)
                                            : p;
                            return new Pose2d(
                                    pBlue.getTranslation().minus(new Translation2d(
                                            totalDriveMeters, 0)),
                                    pBlue.getRotation());
                        },
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // ----- Step 8: Nudge toward bar -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 8: Nudge toward bar")),
                CmdDriveFieldRelative.forDistance(
                        drivetrain,
                        -AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        0.25),

                // ----- Step 9: Retract climber from L1 -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 9: Retract climber")),
                ClimbWhileHeldCommand.retractFromL1(climber, drivetrain)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // ----- Step 10: Hold climb until end -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 10: Hold climb")),
                new CmdHoldClimbUntilEnd(climber, drivetrain),

                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Complete"))).withName("TestClimb");
    }
}
