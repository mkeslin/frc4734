package frc.robot.Commands;

import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
 */
public class CenterToStationCommand extends Command {
    public PositionTracker m_positionTracker;
    public PhotonVision m_photonVision;
    public CommandSwerveDrivetrain m_drivetrain;
    public CommandXboxController m_driveController;

    private boolean driverInterrupted;

    private final PIDController xController = new PIDController(.3, 0, 0);
    private final PIDController yController = new PIDController(.1, 0, 0);
    private final PIDController omegaController = new PIDController(0.03, 0, 0);

    private double AREA_GOAL = 4.2;
    private double AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 1;
    private double ANGLE_ERROR = 3;

    public Timer t = new Timer();
    public Timer t_tray = new Timer();

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
        m_positionTracker = Objects.requireNonNull(positionTracker, "PositionTracker cannot be null");
        m_photonVision = Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
        m_drivetrain = Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        m_driveController = driveController; // Can be null (optional)

        xController.setTolerance(AREA_ERROR);
        yController.setTolerance(CAMERA_X_OFFSET_ERROR);
        omegaController.setTolerance(ANGLE_ERROR);

        addRequirements(m_photonVision, m_drivetrain);
    }

    /**
     * Initializes the command by starting the timer and setting PID controller setpoints.
     * Resets the driver interruption flag.
     */
    @Override
    public void initialize() {
        t.start();

        xController.setSetpoint(AREA_GOAL);
        yController.setSetpoint(0);
        omegaController.setSetpoint(0);
        driverInterrupted = false;
    }

    /**
     * Executes the command by calculating PID outputs based on vision feedback
     * and applying them to the drivetrain. Monitors the coral tray sensor and
     * starts a timer when coral is detected. Stops movement if no vision targets are detected.
     */
    @Override
    public void execute() {
        var xSpeed = -xController.calculate(m_photonVision.getArea());
        var ySpeed = -yController.calculate(m_photonVision.getX());
        var omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_photonVision.getYaw()));
        if (!m_photonVision.hasTargets()) {
            ySpeed = 0;
            xSpeed = 0;
            omegaSpeed = 0;
        }
        m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
        if (m_driveController != null) {
            m_driveController.povUp().onTrue(Commands.runOnce(() -> {
                driverInterrupted = true;
            }));
        }

        if (m_positionTracker.getCoralInTray() && !t_tray.isRunning()) {
            t_tray.start();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (t.hasElapsed(5)) {
            DataLogManager.log("[CenterToStation] WARN: Command timed out after 5 seconds");
            return true;
        }
        if (driverInterrupted) {
            DataLogManager.log("[CenterToStation] Command interrupted by driver");
            return true;
        }
        if (t_tray.hasElapsed(.75)) {
            DataLogManager.log("[CenterToStation] Coral successfully acquired");
            return true;
        }
        if (!m_photonVision.hasTargets()) {
            DataLogManager.log("[CenterToStation] WARN: Lost AprilTag target");
            return true;
        }
        if (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()) {
            DataLogManager.log("[CenterToStation] Successfully centered to station");
            return true;
        }

        return false;
    }

    /**
     * Called when the command ends. Stops and resets all timers.
     * 
     * @param interrupted true if the command was interrupted, false if it completed normally
     */
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();

        t_tray.stop();
        t_tray.reset();
    }

    /**
     * Debug method to publish calculated speeds to SmartDashboard.
     * Currently unused but kept for debugging purposes.
     * 
     * @param x The X speed component
     * @param y The Y speed component
     * @param omega The rotational speed component
     */
    public void getSpeeds(double x, double y, double omega) {
        SmartDashboard.putNumber("xSpeed", x);
        SmartDashboard.putNumber("ySpeed", y);
        SmartDashboard.putNumber("omegaSpeed", omega);
    }
}