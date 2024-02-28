package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class ElevatorStowCommand extends Command {
    public Elevator m_elevator;
    public double target_val;
    public double start_val;
    public double current_val;
    

    public ElevatorStowCommand(Elevator Elevator, double target) {
        m_elevator = Elevator;
        target_val = target;
        addRequirements(m_elevator);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        current_val = m_elevator.getPivotEncoderValue();
        start_val = m_elevator.getPivotEncoderValue();
    }

    @Override
    public void execute() {
        current_val = m_elevator.getPivotEncoderValue();
        if(current_val > start_val * 0.3) {
            m_elevator.setPivot(0.2);
        } else {
            m_elevator.setPivot(0.25);
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
        m_elevator.StopPivot();
    }
}