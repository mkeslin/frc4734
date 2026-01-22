package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to acquire and aim at the hub using vision or fallback heading.
 * 
 * <p>This command attempts to aim at the hub using vision. If vision is good
 * (low ambiguity, sufficient tags), it computes the heading to the hub from
 * the current pose estimate and the known hub position. Otherwise, it uses a
 * fallback heading.
 * 
 * <p>The command uses CmdSnapToHeading internally with ProfiledFieldCentricFacingAngle
 * to smoothly rotate to the target heading. It completes when the heading is
 * stable for the specified number of frames or when the timeout expires.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Heading is stable for stableFrames consecutive frames</li>
 *   <li>Timeout expires</li>
 * </ul>
 * 
 * @param vision The PhotonVision subsystem
 * @param drivetrain The drivetrain subsystem
 * @param fallbackHeadingDeg Fallback heading in degrees (used if vision is poor)
 * @param stableFrames Number of consecutive frames at target before considered stable
 * @param timeoutSec Maximum time to acquire aim
 */
public class CmdAcquireHubAim extends Command {
    private final PhotonVision vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final double fallbackHeadingDeg;
    private final int stableFrames;
    private final double timeoutSec;
    private final Timer timer = new Timer();
    private Command snapToHeadingCommand;
    private int stableFrameCount = 0;
    private Rotation2d lastHeading = null;

    /**
     * Creates a new CmdAcquireHubAim command.
     * 
     * @param vision The PhotonVision subsystem
     * @param drivetrain The drivetrain subsystem
     * @param fallbackHeadingDeg Fallback heading in degrees
     * @param stableFrames Number of consecutive frames at target before considered stable
     * @param timeoutSec Maximum time to acquire aim
     * @throws NullPointerException if vision or drivetrain is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdAcquireHubAim(
            PhotonVision vision,
            CommandSwerveDrivetrain drivetrain,
            double fallbackHeadingDeg,
            int stableFrames,
            double timeoutSec) {
        this.vision = Objects.requireNonNull(vision, "vision cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.fallbackHeadingDeg = fallbackHeadingDeg;
        this.stableFrames = stableFrames;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;

        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        stableFrameCount = 0;
        lastHeading = null;

        // Determine target heading
        Rotation2d targetHeading = computeTargetHeading();
        
        // Create snap to heading command
        snapToHeadingCommand = new CmdSnapToHeading(
                drivetrain,
                () -> targetHeading,
                AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                timeoutSec);
        snapToHeadingCommand.initialize();
    }

    @Override
    public void execute() {
        if (snapToHeadingCommand != null) {
            snapToHeadingCommand.execute();
        }

        // Check if heading is stable
        Rotation2d currentHeading = drivetrain.getPose().getRotation();
        Rotation2d targetHeading = computeTargetHeading();
        
        double headingError = Math.abs(currentHeading.minus(targetHeading).getRadians());
        headingError = Math.min(headingError, 2 * Math.PI - headingError); // Normalize to [-pi, pi]
        
        if (headingError <= AutoConstants.DEFAULT_ROTATION_TOLERANCE) {
            // Check if heading has been stable
            if (lastHeading != null && Math.abs(currentHeading.minus(lastHeading).getRadians()) < 0.01) {
                stableFrameCount++;
            } else {
                stableFrameCount = 1;
            }
        } else {
            stableFrameCount = 0;
        }
        
        lastHeading = currentHeading;
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }

        if (stableFrameCount >= stableFrames) {
            return true;
        }

        if (snapToHeadingCommand != null && snapToHeadingCommand.isFinished()) {
            // Also check if we're at heading even if snap command finished
            return stableFrameCount >= stableFrames;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (snapToHeadingCommand != null) {
            snapToHeadingCommand.end(interrupted);
        }
    }

    /**
     * Computes the target heading to the hub.
     * Uses vision if available and good, otherwise uses fallback heading.
     * 
     * @return The target heading
     */
    private Rotation2d computeTargetHeading() {
        // Try to get pose from vision
        Optional<org.photonvision.EstimatedRobotPose> estimatedPose = vision.getEstimatedRobotPose();
        
        if (estimatedPose.isPresent()) {
            var pose = estimatedPose.get();
            
            // Check if vision is good enough
            // Note: EstimatedRobotPose may not have ambiguity field in all PhotonVision versions
            // Using tag count as quality metric instead
            int tagCount = getVisibleTagCount();
            if (tagCount >= AutoConstants.DEFAULT_MIN_TARGETS) {
                // Use vision pose to compute heading to hub
                Pose2d currentPose = pose.estimatedPose.toPose2d();
                Translation2d hubTranslation = AutoConstants.HUB_POSE.getTranslation();
                Translation2d robotTranslation = currentPose.getTranslation();
                
                Translation2d toHub = hubTranslation.minus(robotTranslation);
                return new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));
            }
        }
        
        // Fallback to fixed heading
        return Rotation2d.fromDegrees(fallbackHeadingDeg);
    }

    /**
     * Gets the number of visible AprilTags from the latest vision result.
     * 
     * @return The number of visible tags, or 0 if no result available
     */
    private int getVisibleTagCount() {
        var result = vision.getLatestResult();
        if (result != null && result.hasTargets()) {
            return result.getTargets().size();
        }
        return 0;
    }

    /**
     * Factory method to create a command with default stable frames and timeout.
     * 
     * @param vision The PhotonVision subsystem
     * @param drivetrain The drivetrain subsystem
     * @param fallbackHeadingDeg Fallback heading in degrees
     * @return A command that acquires hub aim with default settings
     */
    public static Command create(
            PhotonVision vision,
            CommandSwerveDrivetrain drivetrain,
            double fallbackHeadingDeg) {
        return new CmdAcquireHubAim(
                vision,
                drivetrain,
                fallbackHeadingDeg,
                AutoConstants.DEFAULT_STABLE_FRAMES,
                AutoConstants.DEFAULT_HEADING_TIMEOUT);
    }
}
