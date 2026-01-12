package frc.robot.Commands;

import java.util.Objects;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Logging.RobotLogger;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to center the robot to the station using PhotonVision.
 * Uses PID controllers to align the robot based on camera feedback (area, X offset, and yaw).
 * Monitors the coral tray sensor to detect when coral is successfully acquired.
 * The command completes when the robot reaches the target position, coral is acquired,
 * loses vision targets, times out after 5 seconds, or is interrupted by the driver.
 * 
 * @see PhotonVision
 * @see PositionTracker
 * @see CommandSwerveDrivetrain
 * @see BaseCenterToCommand
 */
public class CenterToStationCommand extends BaseCenterToCommand {
    private static final double TIMEOUT_DURATION = 5.0;
    private static final double CORAL_TRAY_TIMEOUT = 0.75;

    private final PositionTracker m_positionTracker;
    private final Timer t_tray = new Timer();

    /**
     * Creates a new CenterToStationCommand.
     * 
     * @param positionTracker The PositionTracker to monitor coral acquisition
     * @param photonVision The PhotonVision camera subsystem for vision feedback
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption (can be null)
     * @throws NullPointerException if positionTracker, photonVision, or drivetrain is null
     */
    public CenterToStationCommand(PositionTracker positionTracker, PhotonVision photonVision,
            CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
        super(createStationConfig(photonVision, drivetrain, driveController));
        m_positionTracker = Objects.requireNonNull(positionTracker, "PositionTracker cannot be null");
    }

    /**
     * Creates a configuration for the station command with Station-specific parameters.
     * 
     * @param photonVision The PhotonVision camera subsystem
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption
     * @return The configured Config object
     */
    private static Config createStationConfig(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain,
            CommandXboxController driveController) {
        Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");

        Config config = new Config(photonVision, drivetrain, CenterMethod.CAMERA, TIMEOUT_DURATION);
        config.driveController = driveController;
        
        // Station-specific camera method parameters
        config.cameraAreaGoal = 4.2;
        config.cameraYOffset = 0.0;
        config.cameraAreaError = 2.0;
        config.cameraXOffsetError = 1.0;
        config.cameraAngleError = 3.0;
        config.cameraXP = 0.3;
        config.cameraYP = 0.1;
        config.cameraOmegaP = 0.03;
        config.negateCameraSpeeds = true; // Station command negates speeds
        
        return config;
    }

    /**
     * Initializes the command by resetting the coral tray timer.
     */
    @Override
    protected void onInitialize() {
        t_tray.stop();
        t_tray.reset();
    }

    /**
     * Executes the command and monitors the coral tray sensor.
     * Starts a timer when coral is detected in the tray.
     */
    @Override
    protected void onExecute() {
        if (m_positionTracker.getCoralInTray() && !t_tray.isRunning()) {
            t_tray.start();
        }
    }

    /**
     * Checks additional finish conditions, specifically coral acquisition.
     * 
     * @return true if coral has been in tray for the required duration
     */
    @Override
    protected boolean additionalFinishConditions() {
        if (t_tray.hasElapsed(CORAL_TRAY_TIMEOUT)) {
            RobotLogger.log("[CenterToStation] Coral successfully acquired");
            return true;
        }
        return false;
    }

    /**
     * Called when the command ends. Stops and resets the coral tray timer.
     */
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        t_tray.stop();
        t_tray.reset();
    }

    @Override
    protected String getCommandName() {
        return "[CenterToStation]";
    }
}
