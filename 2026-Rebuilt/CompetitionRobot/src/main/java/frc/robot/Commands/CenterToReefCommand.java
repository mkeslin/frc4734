package frc.robot.Commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to center the robot to the reef scoring station using PhotonVision.
 * Supports two centering methods: camera-based (using area, X offset, and yaw) and
 * pose-based (using estimated robot pose from AprilTags). Uses PID controllers to
 * align the robot and completes when the target position is reached, vision is lost,
 * timeout occurs, or driver interrupts.
 * 
 * <p>This command pre-calculates target poses for AprilTags 6-11 and 17-22 during
 * construction, positioning the robot at a fixed distance from each tag.
 * 
 * @see PhotonVision
 * @see CommandSwerveDrivetrain
 * @see BaseCenterToCommand
 */
public class CenterToReefCommand extends BaseCenterToCommand {
    /**
     * Creates a new CenterToReefCommand.
     * 
     * @param photonVision The PhotonVision camera subsystem for vision feedback
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption (can be null)
     * @param timeoutDuration The maximum time in seconds before the command times out (must be > 0)
     * @throws NullPointerException if photonVision or drivetrain is null
     * @throws IllegalArgumentException if timeoutDuration is less than or equal to 0
     */
    public CenterToReefCommand(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain,
            CommandXboxController driveController, double timeoutDuration) {
        super(createReefConfig(photonVision, drivetrain, driveController, timeoutDuration));
    }

    /**
     * Creates a configuration for the reef command with default values.
     * 
     * @param photonVision The PhotonVision camera subsystem
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption
     * @param timeoutDuration The timeout duration in seconds
     * @return The configured Config object
     */
    private static Config createReefConfig(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain,
            CommandXboxController driveController, double timeoutDuration) {
        Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");

        Config config = new Config(photonVision, drivetrain, CenterMethod.CAMERA, timeoutDuration);
        config.driveController = driveController;
        
        // Add tag ranges for pose method (6-11 and 17-22)
        config.addTagRange(6, 11);
        config.addTagRange(17, 22);
        
        // Set distance from AprilTag
        config.distanceFromAprilTag = 0.75;
        
        // Camera method parameters (matching original values)
        config.cameraAreaGoal = 37.0;
        config.cameraYOffset = -6.0;
        config.cameraAreaError = 2.0;
        config.cameraXOffsetError = 0.9;
        config.cameraAngleError = 2.5;
        config.cameraXP = 0.03;
        config.cameraYP = 0.03;
        config.cameraOmegaP = 0.03;
        
        // Pose method parameters (matching original values)
        config.poseXError = 0.06;
        config.poseYError = 0.05;
        config.poseAngleError = 3.0;
        config.poseXP = 1.0;
        config.poseYP = 1.0;
        config.poseOmegaP = 0.03;
        
        return config;
    }

    @Override
    protected String getCommandName() {
        return "[CenterToReef]";
    }
}
