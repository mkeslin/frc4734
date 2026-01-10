// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.Elevator;

// public class ElevatorRaiseToAmpCommand extends Command {
//     public Elevator m_Elevator;
//     public double target_val;
//     public double current_val;
//     public double start_val;
//     public double sign;
    

//     public ElevatorRaiseToAmpCommand(Elevator elevator, double target) {
//         m_Elevator = elevator;
//         target_val = target;
//         addRequirements(m_Elevator);
//     }

//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {
//         current_val = m_Elevator.getExtendEncoderValue();
//         start_val = m_Elevator.getExtendEncoderValue();
//         if(target_val < current_val) {
//             sign = -1;
//         } else {
//             sign = 1;
//         }
//     }

//     @Override
//     public void execute() {
//         current_val = m_Elevator.getExtendEncoderValue();
//         if(Math.abs(current_val) < Math.abs(start_val * 0.75)) {
//             m_Elevator.setExtendRetractMotor(sign * 0.65);
//         } else {
//             m_Elevator.setExtendRetractMotor(sign * 0.85);
//         }
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         return Math.abs(current_val - target_val) < 5;
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean interrupted) {
//         m_Elevator.StopExtendRetract();
//     }
// }