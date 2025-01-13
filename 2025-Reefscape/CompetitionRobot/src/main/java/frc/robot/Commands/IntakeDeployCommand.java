// package frc.robot.Commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.Intake;

// public class IntakeDeployCommand extends Command {
//     public Intake m_intake;
//     public double target_val;
//     public double current_val;
    
//     public Timer t = new Timer();

//     public IntakeDeployCommand(Intake intake, double target) {
//         m_intake = intake;
//         target_val = target;
//         addRequirements(m_intake);
//     }

//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {
//         current_val = m_intake.getEncoderValue();

//         t.start();
//     }

//     @Override
//     public void execute() {
//         current_val = m_intake.getEncoderValue();
//         if(current_val > target_val/2) {
//             m_intake.setPivotMotor(0.05);
//         } else {
//             m_intake.setPivotMotor(0.125);
//         }
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         if (t.hasElapsed(2)) { return true;}

//         return current_val >= target_val;
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean interrupted) {
//         m_intake.stopPivot();

//         t.stop();
//         t.reset();
//     }
// }