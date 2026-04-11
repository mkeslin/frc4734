package frc.robot.Auto.commands;

import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;
import frc.robot.ShooterSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Logging.RobotLogger;
import frc.robot.PathPlanner.AllianceUtils;
import frc.robot.PathPlanner.BlueLandmarks;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Builder class for autonomous routines registered in {@link frc.robot.AutoConfigurator}.
 * Method names mirror the SmartDashboard / shuffleboard auto chooser labels so each
 * routine is easy to locate in code.
 *
 * <table border="1">
 * <caption>Chooser label → builder</caption>
 * <tr><th>Dashboard name</th><th>Method</th></tr>
 * <tr><td>Test - PathPlanner</td><td>{@link #buildTestPathPlanner}</td></tr>
 * <tr><td>Test - Drive and Shoot</td><td>{@link #buildTestDriveAndShoot}</td></tr>
 * <tr><td>Test - Climb</td><td>{@link #buildTestClimb}</td></tr>
 * <tr><td>ClimberAuto (Middle)</td><td>{@link #buildClimberAuto}</td></tr>
 * <tr><td>ShooterAuto (Left | Right)</td><td>{@link #buildShooterAutoThroughCenter}</td></tr>
 * <tr><td>ShooterAuto (Center)</td><td>{@link #buildShooterAutoCenter}</td></tr>
 * </table>
 */
public class AutoRoutines {
    private AutoRoutines() {
        // Utility class - prevent instantiation
    }

    /**
     * SmartDashboard: <strong>Test - PathPlanner</strong>. Seeds at left start and pathfinds to the blue shot pose.
     */
    public static Command buildTestPathPlanner(
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            CommandSwerveDrivetrain drivetrain) {
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        return Commands.sequence(
                CmdSeedOdometryFromStartPose.create(StartPoseId.POS_1, startPoseSuppliers, drivetrain),
                CmdDriveToPose.create(
                        drivetrain,
                        () -> BlueLandmarks.ShotPosition,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT))
                .withName("Test - PathPlanner");
    }

    private static Set<Subsystem> shootSubsystemSet(Shooter shooter, Feeder feeder, Floor floor, DeployableIntake intake) {
        HashSet<Subsystem> set = new HashSet<>();
        set.add(shooter);
        set.add(feeder);
        if (floor != null) {
            set.add(floor);
        }
        if (intake != null) {
            set.add(intake);
        }
        return set;
    }

    /**
     * Stow intake deploy and stop rollers before climber retract to avoid mechanism interference.
     */
    private static Command stowIntakeBeforeClimbRetract(DeployableIntake intake) {
        if (intake == null) {
            return Commands.none();
        }
        return Commands.sequence(
                intake.moveToSetDeployPositionCommand(() -> DeployPosition.STOWED)
                        .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT),
                intake.resetIntakeSpeedCommand())
                .withName("stowIntakeBeforeClimbRetract");
    }

    /**
     * SmartDashboard: <strong>ClimberAuto (Middle)</strong> only — start {@link StartPoseId#POS_2} (center).
     * Prelude matches {@link #buildShooterAutoCenter} ({@code C_StartToShot},
     * {@link AutoConstants#SHOOTER_AUTO_CENTER_SHOOT_DURATION}), then tower align + climb on the
     * <strong>left</strong> tower face ({@link BlueLandmarks#TowerAlignLeftOffset}).
     *
     * @param startPoseSuppliers Map including {@link StartPoseId#POS_2} (evaluated at runtime for alliance)
     * @param fallbackHeadingDeg Passed through to shared prelude (reserved if aim steps return)
     * @param targetSpeedsSupplier Center auto shooter RPS (Shuffleboard / override)
     * @param rpmTol             Tolerance for prelude (reserved in {@link #pathToShotThenShoot})
     * @param drivetrain         Drivetrain
     * @param vision             Vision
     * @param shooter            Shooter
     * @param feeder             Feeder
     * @param floor              Optional floor
     * @param intake             Optional intake
     * @param climber            Climber
     */
    public static Command buildClimberAuto(
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            double fallbackHeadingDeg,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
            double rpmTol,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake,
            Climber climber) {

        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        Objects.requireNonNull(climber, "climber cannot be null");
        Objects.requireNonNull(targetSpeedsSupplier, "targetSpeedsSupplier cannot be null");

        Supplier<Double> centerShootDuration = () -> AutoConstants.SHOOTER_AUTO_CENTER_SHOOT_DURATION;
        return Commands.sequence(
                pathToShotThenShoot(
                        StartPoseId.POS_2,
                        startPoseSuppliers,
                        "C_StartToShot",
                        fallbackHeadingDeg,
                        targetSpeedsSupplier,
                        rpmTol,
                        centerShootDuration,
                        drivetrain,
                        vision,
                        shooter,
                        feeder,
                        floor,
                        intake),
                climberAutoTowerAndClimbSequence(
                        () -> BlueLandmarks.TowerAlignLeftOffset,
                        vision,
                        drivetrain,
                        climber,
                        intake))
                .withName("ClimberAuto");
    }

    /**
     * ClimberAuto steps after the preload shot: tower align, climb L1, bar acquire, stow intake, retract, hold.
     */
    private static Command climberAutoTowerAndClimbSequence(
            Supplier<Pose2d> towerAlignPose,
            PhotonVision vision,
            CommandSwerveDrivetrain drivetrain,
            Climber climber,
            DeployableIntake intake) {
        final Pose2d[] poseBeforeDriveToBar = new Pose2d[1];
        return Commands.sequence(
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
                        AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        -AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        0.25),

                // ----- Step 14: Stow intake (before retract — avoids clash with extended climber) -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 14: Stow intake")),
                stowIntakeBeforeClimbRetract(intake),

                // ----- Step 15: Retract climber from L1 -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 15: Retract climber")),
                ClimbWhileHeldCommand.retractFromL1(climber, drivetrain)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // ----- Step 16: Hold climb until end -----
                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Step 16: Hold climb")),
                new CmdHoldClimbUntilEnd(climber, drivetrain),

                Commands.runOnce(() -> RobotLogger.log("[ClimberAuto] Complete")));
    }

    /**
     * SmartDashboard: <strong>ShooterAuto (Left)</strong> or <strong>ShooterAuto (Right)</strong> — serpentine through center and second shot.
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
     * @param targetSpeedsSupplier Supplier of per-motor shooter speeds (left, center, right RPS)
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
    public static Command buildShooterAutoThroughCenter(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathToShot,
            String pathToCenter,
            String pathBackToShot,
            double fallbackHeadingDeg,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
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
        Objects.requireNonNull(targetSpeedsSupplier, "targetSpeedsSupplier cannot be null");
        Objects.requireNonNull(shootDurationSupplier, "shootDurationSupplier cannot be null");

        return Commands.sequence(
                // ----- Step 1: Seed odometry -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 1: Seed odometry", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // ----- Step 2: Lower intake + spin up shooter (parallel) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 2: Lower intake + spin up shooter (parallel)",
                        edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                Commands.parallel(
                        intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                                .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT),
                        Commands.deadline(
                                Commands.waitSeconds(AutoConstants.SHOOTER_SPINUP_DURATION_LEFT_RIGHT),
                                new CmdShooterSpinUp(shooter, targetSpeedsSupplier))),

                // ----- Step 3: Tag snap (webcam unblocked after intake deploy) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 3: Tag snap", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // Step 5 (Acquire hub aim) commented out in current config.

                // ----- Step 6: Shoot (duration from Shuffleboard) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 6: Shoot (duration=%.1fs)",
                        edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), shootDurationSupplier.get()))),
                new DeferredCommand(
                        () -> new ParallelCommandGroup(
                                CmdShootForTime.create(shooter, feeder, floor, shootDurationSupplier.get(), 0.0, targetSpeedsSupplier),
                                intake.shootDeployAgitateWithIntakeRollCommand()),
                        shootSubsystemSet(shooter, feeder, floor, intake)),

                // ----- Step 7: Drive through center (intake on) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 7: Drive through center (intake on)",
                        edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                Commands.deadline(
                        new CmdFollowPath(pathToCenter, AutoConstants.AUTO_PATH_THROUGH_CENTER_TIMEOUT_SEC, drivetrain,
                                AutoConstants.AUTO_PATH_THROUGH_CENTER_MAX_VELOCITY_MPS,
                                AutoConstants.AUTO_PATH_THROUGH_CENTER_MAX_ACCELERATION_MPS2),
                        CmdIntakeOn.create(intake))
                        .andThen(intake.resetIntakeSpeedCommand()),

                // ----- Step 8: Drive back to shot (spin up shooter; slow for testing) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 8: Drive back to shot (spin up)", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                Commands.deadline(
                        new CmdFollowPath(pathBackToShot, AutoConstants.AUTO_PATH_THROUGH_CENTER_TIMEOUT_SEC, drivetrain,
                                AutoConstants.AUTO_PATH_THROUGH_CENTER_MAX_VELOCITY_MPS,
                                AutoConstants.AUTO_PATH_THROUGH_CENTER_MAX_ACCELERATION_MPS2),
                        new CmdShooterSpinUp(shooter, targetSpeedsSupplier)),

                // ----- Step 9: Shoot second load -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 9: Shoot second load (duration=%.1fs)", edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), shootDurationSupplier.get()))),
                new DeferredCommand(
                        () -> new ParallelCommandGroup(
                                CmdShootForTime.create(shooter, feeder, floor, shootDurationSupplier.get(), 0.0, targetSpeedsSupplier),
                                intake.shootDeployAgitateWithIntakeRollCommand()),
                        shootSubsystemSet(shooter, feeder, floor, intake)),

                // ----- Step 10: Raise intake (reset for next run) -----
                // Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 10: Raise intake")),
                // intake.moveToSetDeployPositionCommand(() -> DeployPosition.STOWED)
                //         .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT),

                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Complete", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))))
                .withName("ShooterAutoThroughCenter");
    }

    /**
     * Path to shot pose, deploy intake, shoot once (no center run). Shared by
     * {@link #buildTestDriveAndShoot} and {@link #buildShooterAutoCenter}.
     */
    private static Command pathToShotThenShoot(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathToShot,
            double fallbackHeadingDeg,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
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
        Objects.requireNonNull(targetSpeedsSupplier, "targetSpeedsSupplier cannot be null");
        Objects.requireNonNull(shootDurationSupplier, "shootDurationSupplier cannot be null");

        return Commands.sequence(
                // ----- Step 1: Seed odometry -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 1: Seed odometry", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // ----- Step 2: Tag snap -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 2: Tag snap", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // ----- Step 3: Drive to shot pose (parallel: spin up shooter) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 3: Drive to shot + spin up", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                Commands.deadline(
                        new CmdFollowPath(pathToShot, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain,
                                AutoConstants.AUTO_PATH_CENTER_TO_SHOT_MAX_VELOCITY_MPS,
                                AutoConstants.AUTO_PATH_CENTER_TO_SHOT_MAX_ACCELERATION_MPS2),
                        new CmdShooterSpinUp(shooter, targetSpeedsSupplier)),

                // ----- Step 4: Deploy intake -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 4: Deploy intake (intake=%s)", edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), intake != null ? "present" : "null"))),
                intake != null
                        ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                                .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                                .beforeStarting(Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 4: Intake deploy starting", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))))
                                .finallyDo(interrupted -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 4: Intake deploy finished (interrupted=%b)", edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), interrupted)))
                        : Commands.none(),

                // ----- Step 5: Shoot (intake rolls + deploy agitate during shoot; single command avoids intake conflict) -----
                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Step 5: Shoot", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))),
                Commands.deadline(
                        new DeferredCommand(
                                () -> CmdShootForTime.create(shooter, feeder, floor, shootDurationSupplier.get(), 0.0, targetSpeedsSupplier),
                                floor != null ? Set.of(shooter, feeder, floor) : Set.of(shooter, feeder)),
                        intake != null ? intake.shootDeployAgitateWithIntakeRollCommand() : Commands.none())
                        .andThen(intake != null ? intake.resetIntakeSpeedCommand() : Commands.none()),

                // ----- Step 7: Raise intake (reset for next run) -----
                // Commands.runOnce(() -> RobotLogger.log("[ShooterAuto] Step 7: Raise intake")),
                // intake != null
                //         ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.STOWED)
                //                 .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                //         : Commands.none(),

                Commands.runOnce(() -> RobotLogger.log(String.format("[ShooterAuto] t=%.2f Complete", edu.wpi.first.wpilibj.Timer.getFPGATimestamp()))));
    }

    /**
     * SmartDashboard: <strong>Test - Drive and Shoot</strong>. Seed, path to shot, deploy intake, shoot (no center run).
     *
     * @param id                 Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers
     * @param pathToShot         Path name to shooting position
     * @param fallbackHeadingDeg Fallback heading for hub aiming if aim steps are re-enabled
     * @param targetSpeedsSupplier Per-motor RPS for shooter
     * @param rpmTol             RPM tolerance
     * @param shootDurationSupplier Shoot duration (seconds); e.g. Shuffleboard tuning
     * @param drivetrain         Drivetrain subsystem
     * @param vision             Vision subsystem
     * @param shooter            Shooter subsystem
     * @param feeder             Feeder subsystem
     * @param floor              Optional floor subsystem
     * @param intake             Optional deployable intake
     */
    public static Command buildTestDriveAndShoot(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathToShot,
            double fallbackHeadingDeg,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
            double rpmTol,
            Supplier<Double> shootDurationSupplier,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake) {
        return pathToShotThenShoot(
                id,
                startPoseSuppliers,
                pathToShot,
                fallbackHeadingDeg,
                targetSpeedsSupplier,
                rpmTol,
                shootDurationSupplier,
                drivetrain,
                vision,
                shooter,
                feeder,
                floor,
                intake)
                .withName("Test - Drive and Shoot");
    }

    /**
     * SmartDashboard: <strong>ShooterAuto (Center)</strong>. {@code C_StartToShot}, center start, fixed shoot duration from {@link AutoConstants#SHOOTER_AUTO_CENTER_SHOOT_DURATION}.
     */
    public static Command buildShooterAutoCenter(
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            Supplier<ShooterSpeeds> targetSpeedsSupplier,
            double rpmTol,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake) {
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Supplier<Double> shootDurationCenter = () -> AutoConstants.SHOOTER_AUTO_CENTER_SHOOT_DURATION;
        return pathToShotThenShoot(
                StartPoseId.POS_2,
                startPoseSuppliers,
                "C_StartToShot",
                AutoConstants.DEFAULT_FALLBACK_HEADING_DEG,
                targetSpeedsSupplier,
                rpmTol,
                shootDurationCenter,
                drivetrain,
                vision,
                shooter,
                feeder,
                floor,
                intake)
                .withName("ShooterAuto (Center)");
    }

    /**
     * SmartDashboard: <strong>Test - Climb</strong>. Seed, drive to tower, extend L1, drive to bar,
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
     * @param intake             Optional intake (stowed before climb retract when non-null)
     */
    public static Command buildTestClimb(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            Supplier<Pose2d> towerAlignPose,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Climber climber,
            DeployableIntake intake) {
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
                        AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        -AutoConstants.CLIMB_DRIVE_BEFORE_RETRACT_METERS,
                        0.25),

                // ----- Step 9: Stow intake before climb retract -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 9: Stow intake")),
                stowIntakeBeforeClimbRetract(intake),

                // ----- Step 10: Retract climber from L1 -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 10: Retract climber")),
                ClimbWhileHeldCommand.retractFromL1(climber, drivetrain)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // ----- Step 11: Hold climb until end -----
                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Step 11: Hold climb")),
                new CmdHoldClimbUntilEnd(climber, drivetrain),

                Commands.runOnce(() -> RobotLogger.log("[TestClimb] Complete"))).withName("Test - Climb");
    }
}
