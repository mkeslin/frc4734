package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToTargetCommand extends Command {
    public Limelight m_limelight;
    public CommandSwerveDrivetrain m_drivetrain;

    private double MAX_WHEEL_STRAFE = 0.75;
    private double MAX_WHEEL_ROTATE = 0.5;
    private double MAX_CAMERA_X = 30;

    private double FINAL_AREA = 9;
    private double FINAL_X_OFFSET = 2;
    private double FINAL_ANGLE_DEGREES = 3;

    public double target_x_offset;
    private double target_yaw;
    private double target_area;

    public Timer t = new Timer();

    public CenterToTargetCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;

        addRequirements(m_limelight, m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();
    }

    @Override
    public void execute() {
        target_area = m_limelight.getArea();
        target_yaw = m_limelight.getYaw();
        target_x_offset = m_limelight.getX();
        //wheelRotate = MAX_WHEEL_ROTATE * Math.sin(Math.PI * (target_yaw * 2/Math.PI));
        var forwardDistance = (target_area <= FINAL_AREA && target_area > 0) ? 0.5 : 0;
        var strafeDistance = MAX_WHEEL_STRAFE * Math.sin(Math.PI * (target_x_offset/MAX_CAMERA_X + 1));
        var rotateDistance = (Math.abs(target_yaw * 180/Math.PI) >= FINAL_ANGLE_DEGREES) ? MAX_WHEEL_ROTATE * -Math.signum(target_yaw) : 0;

        m_drivetrain.moveRelative(forwardDistance, strafeDistance, rotateDistance);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        var area = m_limelight.getArea();
        var x_offset = Math.abs(m_limelight.getX());
        var yaw_degrees = Math.abs(m_limelight.getYaw() * 180/Math.PI);
        // use time as failsafe
        return t.hasElapsed(5) || (area > FINAL_AREA && x_offset < FINAL_X_OFFSET && yaw_degrees < FINAL_ANGLE_DEGREES);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
    }
}
