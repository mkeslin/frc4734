package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Cameras.Limelight;

import static frc.robot.Constants.Constants.INTAKE_SENSOR_DELAY;
import static frc.robot.Constants.Constants.SHOOTER_SENSOR_DELAY;

import edu.wpi.first.wpilibj.Timer;

public class MoveToNoteCommand extends Command {
    public Limelight m_limelight;
    public PathPlanner m_PathPlanner;
    public Intake m_Intake;

    public Timer t = new Timer();
    public Timer t2 = new Timer();
    public Timer t3 = new Timer();

    public MoveToNoteCommand(Limelight limelight, PathPlanner pathPlanner, Intake intake) {
        m_limelight = limelight;
        m_PathPlanner = pathPlanner;
        m_Intake = intake;
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
        if(m_Intake.noteIsSeenIntake() && t2.get() == 0) {
            t2.start();
        }
        if(m_Intake.noteIsSeenShooter() && t3.get() == 0) {
            t3.start();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // set a time failsafe
        if (t.hasElapsed(2) || t2.hasElapsed(INTAKE_SENSOR_DELAY) || t3.hasElapsed(SHOOTER_SENSOR_DELAY)) { return true;}
        
        return (m_limelight.getArea() > 0.05 && Math.abs(m_limelight.getY()) < -16);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();

        t2.stop();
        t2.reset();

        t3.stop();
        t3.reset();
    }
}
