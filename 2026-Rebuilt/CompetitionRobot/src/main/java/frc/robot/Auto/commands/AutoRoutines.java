package frc.robot.Auto.commands;

import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
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
     *   <li>Drives to shot position (parallel: spins up shooter)</li>
     *   <li>Acquires hub aim</li>
     *   <li>Waits for shooter at speed</li>
     *   <li>Shoots preload for specified duration</li>
     *   <li>Drives to tower align pose</li>
     *   <li>Applies tag snap before final alignment</li>
     *   <li>Drives to tower align pose again (fine alignment)</li>
     *   <li>Climbs L1 (with timeout)</li>
     *   <li>Holds climb until auto end</li>
     * </ol>
     * 
     * @param id Start pose identifier
     * @param startPoses Map of start pose IDs to their Pose2d values
     * @param shotPose Target pose for shooting (e.g. Landmarks.OurShotPosition())
     * @param towerAlignPose Supplier of target pose for tower/climb (e.g. from Shuffleboard climb-side chooser)
     * @param fallbackHeadingDeg Fallback heading for hub aiming (degrees)
     * @param targetRpm Target shooter RPM
     * @param rpmTol RPM tolerance for shooter at-speed check
     * @param shootDuration Duration to shoot preload (seconds)
     * @param drivetrain The drivetrain subsystem
     * @param vision The PhotonVision subsystem
     * @param shooter The shooter subsystem
     * @param feeder The feeder subsystem
     * @param climber The climber subsystem
     * @return A command representing the complete climber auto routine
     * @throws NullPointerException if any required parameter is null
     */
    public static Command buildClimberAuto(
            StartPoseId id,
            Map<StartPoseId, Pose2d> startPoses,
            Pose2d shotPose,
            Supplier<Pose2d> towerAlignPose,
            double fallbackHeadingDeg,
            double targetRpm,
            double rpmTol,
            double shootDuration,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            Climber climber) {
        
        Objects.requireNonNull(id, "id cannot be null");
        Objects.requireNonNull(startPoses, "startPoses cannot be null");
        Objects.requireNonNull(shotPose, "shotPose cannot be null");
        Objects.requireNonNull(towerAlignPose, "towerAlignPose cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        Objects.requireNonNull(climber, "climber cannot be null");

        Supplier<Double> rpmSupplier = () -> targetRpm;

        return Commands.sequence(
                // 1. Seed odometry from start pose
                CmdSeedOdometryFromStartPose.create(id, startPoses, drivetrain),

                // 2. Tag snap (if good)
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // 3. Drive to shot pose (parallel: spin up shooter)
                Commands.parallel(
                        new CmdDriveToPose(
                                drivetrain,
                                () -> shotPose,
                                AutoConstants.DEFAULT_XY_TOLERANCE,
                                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                                AutoConstants.DEFAULT_PATH_TIMEOUT),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // 4. Acquire hub aim
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // 5. Wait shooter at speed
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),

                // 6. Shoot for time (preload)
                CmdShootForTime.create(shooter, feeder, shootDuration),

                // 7. Drive to tower align pose (side from Shuffleboard "Climb Side" chooser)
                new CmdDriveToPose(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // 8. Tag snap before final alignment
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // 9. Drive to tower align pose again (fine alignment)
                new CmdDriveToPose(
                        drivetrain,
                        towerAlignPose,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),

                // 10. One climb cycle (extend L1 → retract), then end
                ClimbWhileHeldCommand.ascentToCompletion(climber)
                        .withTimeout(AutoConstants.DEFAULT_CLIMB_TIMEOUT),

                // 11. Hold climb until end
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
     * @param startPoses Map of start pose IDs to their Pose2d values
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
            Map<StartPoseId, Pose2d> startPoses,
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
        Objects.requireNonNull(startPoses, "startPoses cannot be null");
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
                CmdSeedOdometryFromStartPose.create(id, startPoses, drivetrain),

                // 2. Tag snap
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),

                // 3. Follow path to shot (parallel: spin up shooter)
                Commands.parallel(
                        new CmdFollowPath(pathToShot, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        new CmdShooterSpinUp(shooter, rpmSupplier)),

                // 4. Acquire hub aim
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg),

                // 5. Wait shooter at speed
                CmdWaitShooterAtSpeed.create(shooter, rpmSupplier, rpmTol),

                // 6. Shoot for time
                CmdShootForTime.create(shooter, feeder, shootDuration),

                // 7. Follow path to center (parallel: intake on)
                Commands.parallel(
                        new CmdFollowPath(pathToCenter, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        CmdIntakeOn.create(intake)),

                // 8. Intake until count (or time-based)
                CmdIntakeUntilCount.create(intake, positionTracker, targetBallCount),

                // 9. Follow path back to shot (parallel: spin up shooter)
                Commands.parallel(
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
}
