package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Drives the robot forward a fixed distance along its current heading, then stops.
 * Used after tower align so the robot moves into the bar to acquire it before climbing.
 */
public class CmdDriveForward extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double distanceMeters;
    private final double timeoutSec;
    private final Timer timer = new Timer();
    private Command pathfindingCommand;
    private Pose2d targetPose;

    /**
     * @param drivetrain The drivetrain
     * @param distanceMeters Forward distance in meters (positive = forward in robot frame)
     * @param timeoutSec Timeout for the move
     */
    public CmdDriveForward(
            CommandSwerveDrivetrain drivetrain,
            double distanceMeters,
            double timeoutSec) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.distanceMeters = distanceMeters;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be > 0, got " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        pathfindingCommand = null;
        Pose2d current = drivetrain.getPose();
        // Forward in robot frame is +X; no rotation change
        targetPose = current.transformBy(new Transform2d(new Translation2d(distanceMeters, 0), Rotation2d.kZero));
        try {
            pathfindingCommand = drivetrain.moveToPose(targetPose)
                    .withTimeout(timeoutSec)
                    .withName("CmdDriveForward");
            pathfindingCommand.initialize();
        } catch (Exception e) {
            pathfindingCommand = null;
        }
    }

    @Override
    public void execute() {
        if (pathfindingCommand != null) {
            pathfindingCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (pathfindingCommand == null) {
            return true;
        }
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }
        if (pathfindingCommand.isFinished()) {
            return true;
        }
        Pose2d current = drivetrain.getPose();
        double xy = current.getTranslation().minus(targetPose.getTranslation()).getNorm();
        if (xy <= AutoConstants.DEFAULT_XY_TOLERANCE) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (pathfindingCommand != null) {
            pathfindingCommand.end(interrupted);
        }
    }
}
