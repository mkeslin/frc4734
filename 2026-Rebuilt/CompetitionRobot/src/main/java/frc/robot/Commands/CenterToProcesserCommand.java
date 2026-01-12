package frc.robot.Commands;

import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
 */
public class CenterToProcesserCommand extends Command {
    public PhotonVision m_photonVision;
    public CommandSwerveDrivetrain m_drivetrain;
    public CommandXboxController m_driveController;

    private boolean driverInterrupted;

    private final PIDController xController = new PIDController(0.15, 0, 0);
    private final PIDController yController = new PIDController(0.03, 0, 0);
    private final PIDController omegaController = new PIDController(0.03, 0, 0);

    private double AREA_GOAL = 7;
    private double AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 1;
    private double ANGLE_ERROR = 3;

    public Timer t = new Timer();

    /**
     * Creates a new CenterToProcesserCommand.
     * 
     * @param photonVision The PhotonVision camera subsystem for vision feedback
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption (can be null)
     * @throws NullPointerException if photonVision or drivetrain is null
     */
    public CenterToProcesserCommand(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
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
     * and applying them to the drivetrain. Stops movement if no vision targets are detected.
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
        //getSpeeds(xSpeed, ySpeed, omegaSpeed);
        m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
        if(m_driveController != null ) {
            m_driveController.povUp().onTrue(Commands.runOnce(() -> {driverInterrupted = true;}));
        }
    }

    /**
     * Determines if the command should finish.
     * 
     * @return true if the timeout has elapsed, driver interrupted, no targets detected,
     *         or all PID controllers are at their setpoints
     */
    @Override
    public boolean isFinished() {
        return t.hasElapsed(5) || driverInterrupted || !m_photonVision.hasTargets() || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()); //|| (area > FINAL_AREA && x_offset < FINAL_X_OFFSET && yaw_degrees < FINAL_ANGLE_DEGREES);
    }

    /**
     * Called when the command ends. Stops and resets the timer.
     * 
     * @param interrupted true if the command was interrupted, false if it completed normally
     */
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
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
