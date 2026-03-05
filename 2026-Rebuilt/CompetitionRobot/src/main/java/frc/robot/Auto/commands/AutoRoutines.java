package frc.robot.Auto.commands;

import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PositionTracker;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.PathPlanner.AllianceUtils;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Builder class for creating autonomous routines from atomic commands.
 * 
 * <p>This class provides static factory methods that compose atomic commands
 * into complete autonomous routines. It demonstrates command composition
 * patterns including parallel execution, sequencing, and conditional logic.
 * 
 * <p>Example usage:
 * <pre>
 * Command climberAuto = AutoRoutines.buildClimberAuto(
 *     StartPoseId.POS_1,
 *     "PathToShot",
 *     "PathToTower",
 *     towerAlignPose,
 *     180.0
 * );
 * </pre>
 */
public class AutoRoutines {
    private AutoRoutines() {
        // Utility class - prevent instantiation
    }

    /**
     * Builds a climber autonomous routine using drive-to poses (no PathPlanner paths).
     * 
     * <p>This routine:
     * <ol>
     *   <li>Seeds odometry from start pose</li>
     *   <li>Applies tag snap if vision quality is good</li>
     *   <li>Drives to shot position (midpoint between start and tower align; parallel: spins up shooter)</li>
     *   <li>Lowers intake if present (deploy so webcam is not blocked)</li>
     *   <li>Acquires hub aim</li>
     *   <li>Waits for shooter at speed</li>
     *   <li>Shoots preload for specified duration</li>
     *   <li>Drives to tower align pose (offset 2 ft toward field center from bar)</li>
     *   <li>Applies tag snap before final alignment</li>
     *   <li>Drives to tower align pose again (fine alignment)</li>
     *   <li>Extends climber to L1 at offset pose</li>
     *   <li>Drives 2 ft toward the bar (field-relative) to acquire the bar</li>
     *   <li>Retracts climber from L1</li>
     *   <li>Holds climb until auto end</li>
     * </ol>
     * 
     * @param id Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers (evaluated at runtime for correct alliance)
     * @param shotPoseSupplier Supplier of target pose for shooting (e.g. midpoint of start and tower align)
     * @param towerAlignPose Supplier of target pose for tower/climb (e.g. from Shuffleboard climb-side chooser)
     * @param fallbackHeadingDeg Fallback heading for hub aiming (degrees)
     * @param targetRpm Target shooter RPM
     * @param rpmTol RPM tolerance for shooter at-speed check
     * @param shootDuration Duration to shoot preload (seconds)
     * @param drivetrain The drivetrain subsystem
     * @param vision The PhotonVision subsystem
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param intake Optional deployable intake; if non-null, intake is lowered before hub aim so webcam is unblocked
     * @param climber The climber subsystem
     * @return A command representing the complete climber auto routine
     * @throws NullPointerException if any required parameter is null
     */
    public static Command buildClimberAuto(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            Supplier<Pose2d> shotPoseSupplier,
            Supplier<Pose2d> towerAlignPose,
            double fallbackHeadingDeg,
            double targetRpm,
            double rpmTol,
            double shootDuration,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
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

        Supplier<Double> rpmSupplier = () -> targetRpm;

        // Captured after extending climber; used by drive-to-bar step (pathfind to pose 2 ft toward bar).
        final Pose2d[] poseBeforeDriveToBar = new Pose2d[1];

        return Commands.sequence(
                // 1. Seed odometry from start pose
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // 2. Tag snap (if good)
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // 3. Drive to shot pose (shooter spins up until drive finishes)
                Commands.deadline(
                        CmdDriveToPose.create(
                                drivetrain,
                                shotPoseSupplier,
                                AutoConstants.DEFAULT_XY_TOLERANCE,
                                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                                AutoConstants.DEFAULT_PATH_TIMEOUT),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // 4. Lower intake so webcam is not blocked (skip if no intake)
                intake != null
                        ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                                .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                        : Commands.none(),

                // 5. Acquire hub aim
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // 6. Wait shooter at speed
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),

                // 7. Shoot for time (preload)
                CmdShootForTime.create(shooter, feeder, shootDuration),

                // 8. Drive to tower align pose (side from Shuffleboard "Climb Side" chooser)
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // 9. Tag snap before final alignment
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // 10. Drive to tower align pose again (fine alignment; pose is 2 ft toward center from bar)
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // 11. Extend climber to L1 at offset pose (no retract yet)
                ClimbWhileHeldCommand.extendL1Only(climber)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // 12. Drive 2 ft toward the bar (field -X) to acquire the bar
                Commands.runOnce(() -> poseBeforeDriveToBar[0] = drivetrain.getPose()),
                CmdDriveToPose.create(
                        drivetrain,
                        () -> {
                            Pose2d p = poseBeforeDriveToBar[0];
                            if (p == null) return null;
                            // Target is 2 ft toward bar (in blue: -X). Convert to blue for pathfinding.
                            Pose2d pBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                    ? AllianceUtils.redToBlue(p) : p;
                            return new Pose2d(
                                    pBlue.getTranslation().minus(new Translation2d(AutoConstants.CLIMB_DRIVE_TO_BAR_METERS, 0)),
                                    pBlue.getRotation());
                        },
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // 13. Retract climber from L1
                ClimbWhileHeldCommand.retractFromL1(climber)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // 14. Hold climb until end
                new CmdHoldClimbUntilEnd(climber, drivetrain)
        ).withName("ClimberAuto");
    }

    /**
     * Builds a shooter autonomous routine.
     * 
     * <p>This routine:
     * <ol>
     *   <li>Seeds odometry from start pose</li>
     *   <li>Applies tag snap if vision quality is good</li>
     *   <li>Follows path to shot position (parallel: spins up shooter)</li>
     *   <li>Acquires hub aim</li>
     *   <li>Waits for shooter at speed</li>
     *   <li>Shoots for specified duration</li>
     *   <li>Follows path to center (parallel: intake on)</li>
     *   <li>Intakes until ball count reached (or timeout)</li>
     *   <li>Follows path back to shot (parallel: spin up shooter)</li>
     *   <li>Acquires hub aim</li>
     *   <li>Waits for shooter at speed</li>
     *   <li>Shoots for specified duration</li>
     * </ol>
     * 
     * @param id Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers (evaluated at runtime for correct alliance)
     * @param pathToShot Path name to shooting position
     * @param pathToCenter Path name to center/field position
     * @param pathBackToShot Path name back to shooting position
     * @param fallbackHeadingDeg Fallback heading for hub aiming (degrees)
     * @param targetRpm Target shooter RPM
     * @param rpmTol RPM tolerance for shooter at-speed check
     * @param shootDuration Duration to shoot (seconds)
     * @param targetBallCount Target ball count for intake
     * @param drivetrain The drivetrain subsystem
     * @param vision The PhotonVision subsystem
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param intake The intake subsystem
     * @param positionTracker The position tracker for sensor reading
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
            double targetRpm,
            double rpmTol,
            double shootDuration,
            int targetBallCount,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            PositionTracker positionTracker) {
        
        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(pathToShot, "pathToShot cannot be null");
        Objects.requireNonNull(pathToCenter, "pathToCenter cannot be null");
        Objects.requireNonNull(pathBackToShot, "pathBackToShot cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        Objects.requireNonNull(intake, "intake cannot be null");
        Objects.requireNonNull(positionTracker, "positionTracker cannot be null");

        Supplier<Double> rpmSupplier = () -> targetRpm;

        return Commands.sequence(
                // 1. Seed odometry
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),

                // 2. Tag snap
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // 3. Follow path to shot (shooter spins up until path finishes)
                Commands.deadline(
                        new CmdFollowPath(pathToShot, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // 4. Acquire hub aim
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // 5. Wait shooter at speed
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),

                // 6. Shoot for time
                CmdShootForTime.create(shooter, feeder, shootDuration),

                // 7. Follow path to center (intake on until path finishes)
                Commands.deadline(
                        new CmdFollowPath(pathToCenter, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        CmdIntakeOn.create(intake)),

                // 8. Intake until count (or time-based)
                CmdIntakeUntilCount.create(intake, positionTracker, targetBallCount),

                // 9. Follow path back to shot (shooter spins up until path finishes)
                Commands.deadline(
                        new CmdFollowPath(pathBackToShot, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // 10. Acquire hub aim
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // 11. Wait shooter at speed
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),

                // 12. Shoot for time
                CmdShootForTime.create(shooter, feeder, shootDuration)
        ).withName("ShooterAuto");
    }

    /**
     * Builds a test routine: seed, drive to shot, lower intake, aim, shoot (no center run).
     * Subset of ShooterAuto for testing drive + shoot flow.
     *
     * @param id Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers
     * @param pathToShot Path name to shooting position
     * @param fallbackHeadingDeg Fallback heading for hub aiming
     * @param targetRpm Target shooter RPM
     * @param rpmTol RPM tolerance
     * @param shootDuration Duration to shoot
     * @param drivetrain Drivetrain subsystem
     * @param vision Vision subsystem
     * @param shooter Shooter subsystem
     * @param feeder Feeder subsystem
     * @param intake Optional deployable intake; if non-null, lowered before hub aim so webcam is unblocked
     */
    public static Command buildTestDriveAndShoot(
            StartPoseId id,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathToShot,
            double fallbackHeadingDeg,
            double targetRpm,
            double rpmTol,
            double shootDuration,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake) {
        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(pathToShot, "pathToShot cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");

        Supplier<Double> rpmSupplier = () -> targetRpm;

        return Commands.sequence(
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),
                Commands.deadline(
                        new CmdFollowPath(pathToShot, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),
                intake != null
                        ? intake.moveToSetDeployPositionCommand(() -> DeployPosition.DEPLOYED)
                                .withTimeout(AutoConstants.DEFAULT_INTAKE_DEPLOY_TIMEOUT)
                        : Commands.none(),
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),
                CmdShootForTime.create(shooter, feeder, shootDuration)
        ).withName("TestDriveAndShoot");
    }

    /**
     * Builds a test routine: seed, drive to tower, extend L1, drive to bar, retract.
     * Subset of ClimberAuto for testing climb flow.
     *
     * @param id Start pose identifier
     * @param startPoseSuppliers Map of start pose IDs to suppliers
     * @param towerAlignPose Supplier of tower align pose (from climb-side chooser)
     * @param drivetrain Drivetrain subsystem
     * @param vision Vision subsystem
     * @param climber Climber subsystem
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
                CmdSeedOdometryFromStartPose.create(id, startPoseSuppliers, drivetrain),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),
                CmdDriveToPose.create(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),
                ClimbWhileHeldCommand.extendL1Only(climber)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),
                Commands.runOnce(() -> poseBeforeDriveToBar[0] = drivetrain.getPose()),
                CmdDriveToPose.create(
                        drivetrain,
                        () -> {
                            Pose2d p = poseBeforeDriveToBar[0];
                            if (p == null) return null;
                            Pose2d pBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                    ? AllianceUtils.redToBlue(p) : p;
                            return new Pose2d(
                                    pBlue.getTranslation().minus(new Translation2d(AutoConstants.CLIMB_DRIVE_TO_BAR_METERS, 0)),
                                    pBlue.getRotation());
                        },
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),
                ClimbWhileHeldCommand.retractFromL1(climber)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),
                new CmdHoldClimbUntilEnd(climber, drivetrain)
        ).withName("TestClimb");
    }
}
