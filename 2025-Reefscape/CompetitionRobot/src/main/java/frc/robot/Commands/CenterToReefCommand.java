package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToReefCommand extends Command {
    public Limelight m_limelight;
    public CommandSwerveDrivetrain m_drivetrain;

    private final PIDController xController = new PIDController(0.05, 0, 0);
    private final PIDController yController = new PIDController(0.05, 0, 0);
    private final PIDController omegaController = new PIDController(0.05, 0, 0);

    private double AREA_GOAL = 30;
    private double AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 1;
    private double ANGLE_ERROR = 5;

    public Timer t = new Timer();

    public CenterToReefCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;

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
    }

    @Override
    public void execute() {

        if(m_limelight.hasTargets()) {
            //possibly add additional conditions for specific target IDs
            
            var xSpeed = xController.calculate(m_limelight.getArea());
            if (xController.atSetpoint()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(m_limelight.getX());
            if (yController.atSetpoint()) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_limelight.getYaw()));
            if (omegaController.atSetpoint()) {
                omegaSpeed = 0;
            }
            //getSpeeds(xSpeed, ySpeed, omegaSpeed);
            m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return t.hasElapsed(5) || !m_limelight.hasTargets() || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()); //|| (area > FINAL_AREA && x_offset < FINAL_X_OFFSET && yaw_degrees < FINAL_ANGLE_DEGREES);
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