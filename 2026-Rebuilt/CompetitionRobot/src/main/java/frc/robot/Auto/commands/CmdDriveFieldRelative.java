package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Drives at a fixed field-relative velocity for a fixed duration.
 * Bypasses pathfinding for short moves that fall within a single nav grid cell.
 *
 * <p>Velocities are in blue alliance frame: +X toward red, +Y toward left wall.
 * On red alliance, signs are flipped so the robot moves in the same physical direction.
 */
public class CmdDriveFieldRelative extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    /** Field-relative vx (m/s) in blue frame. Negative = toward blue wall. */
    private final double vxBlue;
    /** Field-relative vy (m/s) in blue frame. Positive = toward left wall. */
    private final double vyBlue;
    private final double durationSec;
    private final Timer timer = new Timer();

    /**
     * @param drivetrain   The drivetrain
     * @param vxBlue      Field-relative X velocity (m/s) in blue frame
     * @param vyBlue      Field-relative Y velocity (m/s) in blue frame
     * @param durationSec Duration to drive (seconds)
     */
    public CmdDriveFieldRelative(
            CommandSwerveDrivetrain drivetrain,
            double vxBlue,
            double vyBlue,
            double durationSec) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.vxBlue = vxBlue;
        this.vyBlue = vyBlue;
        this.durationSec = durationSec;
        addRequirements(drivetrain);
    }

    /**
     * Creates a command that drives a fixed distance in field coordinates (blue frame).
     * Uses constant velocity for the calculated duration.
     *
     * @param drivetrain   The drivetrain
     * @param distanceXBlue Distance in X (m); negative = toward blue wall
     * @param distanceYBlue Distance in Y (m); positive = toward left wall
     * @param speedMps     Speed for the move (m/s)
     */
    public static Command forDistance(
            CommandSwerveDrivetrain drivetrain,
            double distanceXBlue,
            double distanceYBlue,
            double speedMps) {
        if (Math.abs(speedMps) < 0.01) {
            return Commands.none();
        }
        double totalDist = Math.hypot(distanceXBlue, distanceYBlue);
        if (totalDist < 0.001) {
            return Commands.none();
        }
        double duration = totalDist / Math.abs(speedMps);
        double vx = (distanceXBlue / totalDist) * speedMps;
        double vy = (distanceYBlue / totalDist) * speedMps;
        return new CmdDriveFieldRelative(drivetrain, vx, vy, duration);
    }

    /**
     * Field-relative move along blue X only (vy = 0 for the whole command). Prefer this over
     * {@link #forDistance} with dy=0 so duration and velocity are not tied to a combined hypot path.
     */
    public static Command forDistanceXOnly(
            CommandSwerveDrivetrain drivetrain, double distanceXBlue, double speedMps) {
        if (Math.abs(speedMps) < 0.01) {
            return Commands.none();
        }
        double dist = Math.abs(distanceXBlue);
        if (dist < 1e-4) {
            return Commands.none();
        }
        double duration = dist / Math.abs(speedMps);
        double vx = Math.copySign(Math.abs(speedMps), distanceXBlue);
        return new CmdDriveFieldRelative(drivetrain, vx, 0.0, duration);
    }

    /**
     * Field-relative move along blue Y only (vx = 0 for the whole command).
     */
    public static Command forDistanceYOnly(
            CommandSwerveDrivetrain drivetrain, double distanceYBlue, double speedMps) {
        if (Math.abs(speedMps) < 0.01) {
            return Commands.none();
        }
        double dist = Math.abs(distanceYBlue);
        if (dist < 1e-4) {
            return Commands.none();
        }
        double duration = dist / Math.abs(speedMps);
        double vy = Math.copySign(Math.abs(speedMps), distanceYBlue);
        return new CmdDriveFieldRelative(drivetrain, 0.0, vy, duration);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(durationSec)) {
            return;
        }
        double vx = vxBlue;
        double vy = vyBlue;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            vx = -vxBlue;
            vy = -vyBlue;
        }
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, 0, drivetrain.getPose().getRotation());
        drivetrain.applyChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSec);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.stop();
    }
}
