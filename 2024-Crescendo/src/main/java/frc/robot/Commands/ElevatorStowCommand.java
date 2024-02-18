package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class ElevatorStowCommand extends Command {
    public Elevator m_Elevator;
    public double target_val;
    public double start_val;
    public double current_val;
    

    public ElevatorStowCommand(Elevator Elevator, double target) {
        m_Elevator = Elevator;
        target_val = target;
        addRequirements(m_Elevator);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        current_val = m_Elevator.getPivotEncoderValue();
        start_val = m_Elevator.getPivotEncoderValue();
    }

    @Override
    public void execute() {
        current_val = m_Elevator.getPivotEncoderValue();
        if(current_val > start_val/2) {
            m_Elevator.setPivot(0.1);
        } else {
            m_Elevator.setPivot(0.2);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return current_val >= target_val;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_Elevator.StopPivot();
    }
}