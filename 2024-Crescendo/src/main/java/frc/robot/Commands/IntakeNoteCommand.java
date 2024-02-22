package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
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
        m_intake.startIn();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // set a time failsafe
        if (t.hasElapsed(2)) { return true;}

        // main finished condition
        return m_intake.noteIsSeen();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        t.stop();
        t.reset();
    }
}
