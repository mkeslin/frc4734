package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.River;

public class IntakeEjectCommand extends Command {

    public Intake m_intake;
    public River m_river;
    public Timer t = new Timer();

    public IntakeEjectCommand(Intake intake, River river) {
        m_intake = intake;
        m_river = river;
        addRequirements(m_intake, m_river);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();
    }

    @Override
    public void execute() {
        m_intake.startOut();
        m_river.startOut();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // set a time failsafe
        return (t.hasElapsed(1));
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        m_river.stopRiver();
        
        t.stop();
        t.reset();
    }
}