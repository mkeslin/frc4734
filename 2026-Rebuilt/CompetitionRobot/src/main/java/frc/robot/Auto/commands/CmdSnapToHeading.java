package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.SwerveDrivetrain.ProfiledFieldCentricFacingAngle;
import frc.robot.SwerveDrivetrain.DrivetrainConstants;

/**
 * Command to snap to a target heading using ProfiledFieldCentricFacingAngle.
 * 
 * <p>This command rotates the robot to face a target heading using the drivetrain's
 * ProfiledFieldCentricFacingAngle request. This provides smooth, motion-profiled
 * rotation without translation. The command completes when the heading error is
 * within tolerance or when the timeout expires.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Heading error is within rotTol</li>
 *   <li>Timeout expires</li>
 * </ul>
 * 
 * @param drivetrain The drivetrain subsystem
 * @param headingSupplier Supplier of the target heading
 * @param rotTol Rotation tolerance in radians
 * @param timeoutSec Maximum time to reach the heading
 */
public class CmdSnapToHeading extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Rotation2d> headingSupplier;
    private final double rotTol;
    private final double timeoutSec;
    private final Timer timer = new Timer();
    private final ProfiledFieldCentricFacingAngle facingAngleRequest = new ProfiledFieldCentricFacingAngle(
            new TrapezoidProfile.Constraints(
                    DrivetrainConstants.MaxAngularRate,
                    DrivetrainConstants.kMaxAngularAcceleration));

    /**
     * Creates a new CmdSnapToHeading command.
     * 
     * @param drivetrain The drivetrain subsystem
     * @param headingSupplier Supplier of the target heading
     * @param rotTol Rotation tolerance in radians
     * @param timeoutSec Maximum time to reach the heading
     * @throws NullPointerException if drivetrain or headingSupplier is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdSnapToHeading(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Rotation2d> headingSupplier,
            double rotTol,
            double timeoutSec) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.headingSupplier = Objects.requireNonNull(headingSupplier, "headingSupplier cannot be null");
        this.rotTol = rotTol;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        // Configure request for rotation only (zero translation)
        facingAngleRequest.VelocityX = 0.0;
        facingAngleRequest.VelocityY = 0.0;
        facingAngleRequest.TargetDirection = headingSupplier.get();
    }

    @Override
    public void execute() {
        // Update target direction in case supplier changes
        facingAngleRequest.TargetDirection = headingSupplier.get();
        
        // Apply the request
        drivetrain.setControl(facingAngleRequest);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }

        // Check if heading error is within tolerance
        Rotation2d currentHeading = drivetrain.getPose().getRotation();
        Rotation2d targetHeading = headingSupplier.get();
        double headingError = Math.abs(currentHeading.minus(targetHeading).getRadians());
        
        // Normalize error to [-pi, pi]
        headingError = Math.min(headingError, 2 * Math.PI - headingError);
        
        return headingError <= rotTol;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            drivetrain.stop();
        }
    }

    /**
     * Factory method to create a command with default tolerance and timeout.
     * 
     * @param drivetrain The drivetrain subsystem
     * @param headingSupplier Supplier of the target heading
     * @return A command that snaps to heading with default settings
     */
    public static Command create(CommandSwerveDrivetrain drivetrain, Supplier<Rotation2d> headingSupplier) {
        return new CmdSnapToHeading(
                drivetrain,
                headingSupplier,
                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                AutoConstants.DEFAULT_HEADING_TIMEOUT);
    }
}
