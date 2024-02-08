package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeNoteCommand extends Command {
    public Intake m_intake;
    public Timer t = new Timer();

    public IntakeNoteCommand(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();
    }

    @Override
    public void execute() {
        m_intake.commandStartIn();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return t.hasElapsed(2);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {}
}
