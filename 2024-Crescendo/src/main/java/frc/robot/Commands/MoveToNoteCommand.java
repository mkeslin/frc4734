package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import edu.wpi.first.wpilibj.Timer;

public class MoveToNoteCommand extends Command {
    public Limelight m_limelight;
    public PathPlanner m_PathPlanner;

    public Timer t = new Timer();

    public MoveToNoteCommand(Limelight limelight, PathPlanner pathPlanner) {
        m_limelight = limelight;
        m_PathPlanner = pathPlanner;

        addRequirements(m_limelight, m_PathPlanner);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();
    }

    @Override
    public void execute() {
        m_PathPlanner.moveRelative(1, 0, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // set a time failsafe
        if (t.hasElapsed(2)) { return true;}
        
        return (m_limelight.getArea() > 0.05 && Math.abs(m_limelight.getY()) < -16);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
    }
}
