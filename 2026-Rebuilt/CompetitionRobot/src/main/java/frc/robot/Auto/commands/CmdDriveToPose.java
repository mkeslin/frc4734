package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.PathPlanner.AllianceUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Logging.RobotLogger;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to drive to a target pose using pathfinding.
 *
 * <p>Target pose is always in <b>blue alliance</b> coordinates; pathfinding is
 * run via PathPlanner (pathfindToPoseFlipped) so it works correctly on both red
 * and blue. The pathfinding command is <b>scheduled</b> by the CommandScheduler
 * (via {@link Commands#defer}) so it runs normally and can apply drive output.
 *
 * <p>Completion: robot at target (within tolerances), or timeout.
 */
public final class CmdDriveToPose {

    private CmdDriveToPose() {
        // Factory only
    }

    /**
     * Creates a command that pathfinds to the supplied target pose (blue coordinates).
     * Resolves target at schedule time. Works on both alliances.
     *
     * @param drivetrain           drivetrain subsystem
     * @param targetPoseSupplier   supplier of target pose in blue coordinates
     * @param xyTol                XY tolerance (m)
     * @param rotTol               rotation tolerance (rad)
     * @param timeoutSec           max time to reach pose (s)
     * @return command that drives to pose or times out
     */
    public static Command create(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            double xyTol,
            double rotTol,
            double timeoutSec) {
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(targetPoseSupplier, "targetPoseSupplier cannot be null");
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        return Commands.defer(
                () -> {
                    Pose2d target = targetPoseSupplier.get();
                    if (target == null) {
                        RobotLogger.logError("[CmdDriveToPose] targetPose is null; skipping pathfinding");
                        return Commands.none();
                    }
                    if (!AutoBuilder.isPathfindingConfigured()) {
                        RobotLogger.logError("[CmdDriveToPose] AutoBuilder pathfinding not configured; skipping");
                        return Commands.none();
                    }
                    final double[] startTime = { 0 };
                    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                    Command pathfind;
                    try {
                        pathfind = drivetrain.pathfindToPoseBlue(target)
                                .until(() -> atPose(drivetrain, targetPoseSupplier, xyTol, rotTol))
                                .withTimeout(timeoutSec)
                                .withName("CmdDriveToPose");
                    } catch (Exception e) {
                        RobotLogger.logError("[CmdDriveToPose] pathfindToPoseBlue failed: " + e.getMessage());
                        return Commands.none();
                    }
                    return pathfind
                            .beforeStarting(() -> {
                                try {
                                    startTime[0] = Timer.getFPGATimestamp();
                                    Pose2d current = drivetrain.getPose();
                                    RobotLogger.log(String.format(
                                            "[CmdDriveToPose] started targetBlue=(%.2f, %.2f) current=(%.2f, %.2f) alliance=%s",
                                            target.getX(), target.getY(),
                                            current.getX(), current.getY(),
                                            alliance.name()));
                                    // PathPlanner uses blue nav grid (typically 0–16.54 x 0–8.07); check pose in blue frame.
                                    Pose2d currentBlue = alliance == Alliance.Red ? AllianceUtils.redToBlue(current) : current;
                                    final double maxX = 16.6, maxY = 8.2;
                                    if (currentBlue.getX() < -0.5 || currentBlue.getX() > maxX || currentBlue.getY() < -0.5 || currentBlue.getY() > maxY) {
                                        RobotLogger.logError(String.format(
                                                "[CmdDriveToPose] pose in blue frame (%.2f, %.2f) is outside typical nav grid (0–%.1f, 0–%.1f); pathfinding may not move robot.",
                                                currentBlue.getX(), currentBlue.getY(), maxX, maxY));
                                    }
                                } catch (Exception e) {
                                    RobotLogger.logError("[CmdDriveToPose] beforeStarting: " + e.getMessage());
                                }
                            })
                            .finallyDo(interrupted -> {
                                try {
                                    drivetrain.stop();
                                    double elapsed = Timer.getFPGATimestamp() - startTime[0];
                                    Pose2d current = drivetrain.getPose();
                                    boolean atTarget = atPose(drivetrain, targetPoseSupplier, xyTol, rotTol);
                                    RobotLogger.log(String.format(
                                            "[CmdDriveToPose] ended interrupted=%b elapsed=%.2fs current=(%.2f, %.2f) atPose=%b",
                                            interrupted, elapsed, current.getX(), current.getY(), atTarget));
                                } catch (Exception e) {
                                    RobotLogger.logError("[CmdDriveToPose] finallyDo: " + e.getMessage());
                                }
                            });
                },
                Set.of(drivetrain));
    }

    /**
     * Creates a command with default tolerances and timeout.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        return create(
                drivetrain,
                targetPoseSupplier,
                AutoConstants.DEFAULT_XY_TOLERANCE,
                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                AutoConstants.DEFAULT_POSE_TIMEOUT);
    }

    /**
     * Whether the robot is at the target pose (alliance-relative comparison).
     */
    private static boolean atPose(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> targetBlueSupplier,
            double xyTol,
            double rotTol) {
        Pose2d current = drivetrain.getPose();
        Pose2d targetBlue = targetBlueSupplier.get();
        if (targetBlue == null) {
            return false;
        }
        Pose2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? AllianceUtils.blueToRed(targetBlue)
                : targetBlue;
        Translation2d transErr = current.getTranslation().minus(target.getTranslation());
        double xyErr = transErr.getNorm();
        double rotErr = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());
        return xyErr <= xyTol && rotErr <= rotTol;
    }
}
