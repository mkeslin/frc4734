package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.River;

public class ShootNoteCommand extends Command {
    public Intake m_intake;
    public Shooter m_shooter;
    public River m_river;
    public double m_speed;

    public Timer t = new Timer();

    public ShootNoteCommand(Intake intake, Shooter shooter, River river, double speed) {
        m_intake = intake;
        m_shooter = shooter;
        m_river = river;
        m_speed = speed;
        addRequirements(m_intake, m_shooter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();
    }

    @Override
    public void execute() {
        if(t.get() < 0.1) {
            m_intake.startOut();
        } else {
            m_shooter.shoot(m_speed);
            if(t.get() < 1.4) {
                m_intake.stopRoller();
            }
            else {
                m_intake.startIn(-m_speed);
                m_river.startIn(m_speed);
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return t.hasElapsed(1.7);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        m_shooter.stopShoot();
        m_river.stopRiver();
        
        t.stop();
        t.reset();
    }
}
