package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ShootNoteCommand extends Command {
    public Intake m_intake;
    public Shooter m_shooter;
    public Timer t = new Timer();

    public ShootNoteCommand(Intake intake, Shooter shooter) {
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
        if(t.get() < 0.5) {
            m_intake.startOut();
        } else {
            m_shooter.commandShoot();
            if(t.get() < 2) {
                m_intake.stopRoller();
            }
            else {
                m_intake.startIn();
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return t.hasElapsed(4);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        m_shooter.stop();
        t.stop();
        t.reset();
    }
}
