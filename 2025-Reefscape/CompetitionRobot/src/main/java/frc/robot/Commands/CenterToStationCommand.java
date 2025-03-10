package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToStationCommand extends Command {
    public Limelight m_limelight;
    public CommandSwerveDrivetrain m_drivetrain;
    public CommandXboxController m_driveController;

    private boolean driverInterrupted;

    private final PIDController xController = new PIDController(0.15, 0, 0);
    private final PIDController yController = new PIDController(0.03, 0, 0);
    private final PIDController omegaController = new PIDController(0.03, 0, 0);

    private double AREA_GOAL = 4;
    private double AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 1;
    private double ANGLE_ERROR = 3;

    public Timer t = new Timer();

    public CenterToStationCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        m_driveController = driveController;

        xController.setTolerance(AREA_ERROR);
        yController.setTolerance(CAMERA_X_OFFSET_ERROR);
        omegaController.setTolerance(ANGLE_ERROR);

        addRequirements(m_limelight, m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();

        xController.setSetpoint(AREA_GOAL);
        yController.setSetpoint(0);
        omegaController.setSetpoint(0);
        driverInterrupted = false;
    }

    @Override
    public void execute() {
        var xSpeed = -xController.calculate(m_limelight.getArea());
        var ySpeed = -yController.calculate(m_limelight.getX());
        var omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_limelight.getYaw()));
        if (!m_limelight.hasTargets()) {
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

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return t.hasElapsed(5) || driverInterrupted || !m_limelight.hasTargets() || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()); //|| (area > FINAL_AREA && x_offset < FINAL_X_OFFSET && yaw_degrees < FINAL_ANGLE_DEGREES);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
    }

    public void getSpeeds(double x, double y, double omega) {
        SmartDashboard.putNumber("xSpeed", x);
        SmartDashboard.putNumber("ySpeed", y);
        SmartDashboard.putNumber("omegaSpeed", omega);
    }
}