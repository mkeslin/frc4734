package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PathPlanner.AllianceUtils;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Holds the robot at a pose by pathfinding to it with tight tolerances for a duration.
 *
 * <p>Returns a single command (pathfind to pose with timeout) so it runs under the scheduler
 * and applies drive output. No .until(atPose) so the robot keeps correcting for the full duration.
 *
 * @param durationSec Duration to hold (s); 0 = use a long timeout (effectively until interrupted).
 */
public final class CmdHoldPose {

    private static final double INDEFINITE_TIMEOUT_SEC = 1000.0;

    private CmdHoldPose() {
        // Factory only
    }

    /**
     * Creates a command that pathfinds to the supplied pose (blue coordinates) for the given duration.
     */
    public static Command create(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> poseSupplier,
            double durationSec) {
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(poseSupplier, "poseSupplier cannot be null");
        double timeoutSec = durationSec > 0 ? durationSec : INDEFINITE_TIMEOUT_SEC;
        return Commands.defer(
                () -> {
                    Pose2d target = poseSupplier.get();
                    if (target == null) {
                        return Commands.none();
                    }
                    // PathPlanner expects blue; convert if supplier gave alliance-relative (e.g. getPose()).
                    Pose2d targetBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                            ? AllianceUtils.redToBlue(target)
                            : target;
                    return drivetrain.pathfindToPoseBlue(targetBlue)
                            .withTimeout(timeoutSec)
                            .withName("CmdHoldPose");
                },
                Set.of(drivetrain));
    }
}
