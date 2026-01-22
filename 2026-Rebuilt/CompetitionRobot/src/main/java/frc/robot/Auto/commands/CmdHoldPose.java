package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to hold the robot at a specified pose.
 * 
 * <p>This command maintains the robot's position and orientation at a target pose.
 * If durationSec is 0, the command runs indefinitely until interrupted. Otherwise,
 * it holds for the specified duration.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Duration expires (if durationSec > 0)</li>
 *   <li>Command is interrupted (if durationSec == 0, runs indefinitely)</li>
 * </ul>
 * 
 * @param drivetrain The drivetrain subsystem
 * @param poseSupplier Supplier of the target pose to hold
 * @param durationSec Duration to hold pose (0 = indefinite until interrupted)
 */
public class CmdHoldPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> poseSupplier;
    private final double durationSec;
    private final Timer timer = new Timer();
    private Command holdCommand;

    /**
     * Creates a new CmdHoldPose command.
     * 
     * @param drivetrain The drivetrain subsystem
     * @param poseSupplier Supplier of the target pose to hold
     * @param durationSec Duration to hold pose (0 = indefinite until interrupted)
     * @throws NullPointerException if drivetrain or poseSupplier is null
     */
    public CmdHoldPose(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> poseSupplier,
            double durationSec) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier cannot be null");
        this.durationSec = durationSec;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        // Use pathfinding with tight constraints to hold pose
        // This will continuously pathfind to the target pose, effectively holding it
        Pose2d targetPose = poseSupplier.get();
        if (targetPose != null) {
            // Use very tight tolerances for holding
            holdCommand = new CmdDriveToPose(
                    drivetrain,
                    poseSupplier,
                    0.02, // Very tight XY tolerance (2 cm)
                    0.01, // Very tight rotation tolerance (~0.6 degrees)
                    durationSec > 0 ? durationSec : 1000.0 // Long timeout if indefinite
            );
            holdCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (holdCommand != null) {
            // Reinitialize if pose changes
            Pose2d currentTarget = poseSupplier.get();
            if (currentTarget != null) {
                holdCommand.execute();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (durationSec > 0 && timer.hasElapsed(durationSec)) {
            return true;
        }
        
        // If duration is 0, run indefinitely until interrupted
        if (durationSec == 0) {
            return false;
        }
        
        if (holdCommand != null) {
            return holdCommand.isFinished();
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (holdCommand != null) {
            holdCommand.end(interrupted);
        }
        if (interrupted) {
            drivetrain.stop();
        }
    }
}
