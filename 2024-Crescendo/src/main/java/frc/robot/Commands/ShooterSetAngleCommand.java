package frc.robot.Commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSetAngleCommand extends Command {

    public Shooter m_shooter;
    public TalonFX m_motor;
    public double target_val;
    public double max_val;
    public double start_val;
    public double current_val;
    public MotionMagicVoltage m_request;

    // public Timer t = new Timer();

    public ShooterSetAngleCommand(Shooter shooter, double max) {
        m_shooter = shooter;
        max_val = max;
        m_motor = shooter.getPivotTalon();
        m_request = new MotionMagicVoltage(0);
        addRequirements(m_shooter);
    }

    public void setTarget(double t) {
        target_val = t;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        start_val = m_shooter.getPivotEncoderValue();
        current_val = m_shooter.getPivotEncoderValue();
        // t.start();
    }

    @Override
    public void execute() {
        //m_motor.setControl(m_request.withPosition(target_val));
        current_val = m_shooter.getPivotEncoderValue();
        if(target_val > start_val) {
            if(current_val > start_val + (target_val - start_val)/2 && target_val - start_val > 0.3) {
                m_shooter.setPivotMotor(0.1);
            } else {
                m_shooter.setPivotMotor(0.2);
            }
        } else {
            if(current_val < start_val + (target_val - start_val)/2 && target_val - start_val < 0.3) {
                m_shooter.setPivotMotor(-0.1);
            } else {
                m_shooter.setPivotMotor(-0.2);
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // if (t.hasElapsed(3)) { return true;}

        return (
            (target_val < max_val) ||
            (target_val > 0) ||
            (target_val >= start_val && current_val >= target_val) ||
            (target_val <= start_val && current_val <= target_val)
        );
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_shooter.holdPivot();
        /*if (target_val < 1) {
            m_shooter.stopPivot();
        } else {
            m_shooter.holdPivot();
        }*/
        // t.stop();
        // t.reset();
    }
}
