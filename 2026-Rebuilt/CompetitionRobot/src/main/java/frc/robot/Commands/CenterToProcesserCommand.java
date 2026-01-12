package frc.robot.Commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to center the robot to the processor station using PhotonVision.
 * Uses PID controllers to align the robot based on camera feedback (area, X offset, and yaw).
 * The command completes when the robot reaches the target position, loses vision targets,
 * times out after 5 seconds, or is interrupted by the driver.
 * 
 * @see PhotonVision
 * @see CommandSwerveDrivetrain
 * @see BaseCenterToCommand
 */
public class CenterToProcesserCommand extends BaseCenterToCommand {
    private static final double TIMEOUT_DURATION = 5.0;

    /**
     * Creates a new CenterToProcesserCommand.
     * 
     * @param photonVision The PhotonVision camera subsystem for vision feedback
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption (can be null)
     * @throws NullPointerException if photonVision or drivetrain is null
     */
    public CenterToProcesserCommand(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
        super(createProcessorConfig(photonVision, drivetrain, driveController));
    }

    /**
     * Creates a configuration for the processor command with Processor-specific parameters.
     * 
     * @param photonVision The PhotonVision camera subsystem
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption
     * @return The configured Config object
     */
    private static Config createProcessorConfig(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain,
            CommandXboxController driveController) {
        Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");

        Config config = new Config(photonVision, drivetrain, CenterMethod.CAMERA, TIMEOUT_DURATION);
        config.driveController = driveController;
        
        // Processor-specific camera method parameters
        config.cameraAreaGoal = 7.0;
        config.cameraYOffset = 0.0;
        config.cameraAreaError = 2.0;
        config.cameraXOffsetError = 1.0;
        config.cameraAngleError = 3.0;
        config.cameraXP = 0.15;
        config.cameraYP = 0.03;
        config.cameraOmegaP = 0.03;
        config.negateCameraSpeeds = true; // Processor command negates speeds
        
        return config;
    }

    @Override
    protected String getCommandName() {
        return "[CenterToProcesser]";
    }
}
