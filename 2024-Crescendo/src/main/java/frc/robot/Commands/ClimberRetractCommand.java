package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class ClimberRetractCommand extends Command {
    public Climber m_Climber;
    public double target_val;
    public double start_val;
    public double current_val;
    

    public ClimberRetractCommand(Climber climber, double target) {
        m_Climber = climber;
        target_val = target;
        addRequirements(m_Climber);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        current_val = m_Climber.getEncoderValue();
        
    }

    @Override
    public void execute() {
        current_val = m_Climber.getEncoderValue();
        if(current_val < target_val * 0.75) {
            m_Climber.setClimberMotors(-0.4);
        } else {
            m_Climber.setClimberMotors(-0.5);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return current_val <= target_val;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        //m_Climber.setClimberMotors(-0.04);
        m_Climber.stopClimberMotors();
    }
}