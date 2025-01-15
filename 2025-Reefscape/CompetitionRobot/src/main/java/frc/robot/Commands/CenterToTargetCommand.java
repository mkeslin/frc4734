package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToTargetCommand extends Command {
    public Limelight m_limelight;
    public CommandSwerveDrivetrain m_drivetrain;
    //public int m_target;

    private double MAX_WHEEL_STRAFE = 1;
    private double MAX_CAMERA_X = 30;

    private double offset;
    public double target_offset;
    private double wheelStrafe;

    public Timer t = new Timer();
    //public Timer t2 = new Timer();

    public CenterToTargetCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        //m_target = target; //target should be 0 for Note alignment

        addRequirements(m_limelight, m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        offset = 0;
        t.start();
    }

    @Override
    public void execute() {
        /*if the notes are being tracked and the target area is noticeable (not just a blip),
        or if the AprilTags are being tracked and target is noticeable and the ID is correct*/
        if(m_limelight.getArea() > 0.05 /*&& (m_target < 1 || m_limelight.getAprilTagID() == m_target)*/) {
            //if the target is to the left of the camera's sensor
            if(m_limelight.getX() < 0) {
                SmartDashboard.putString("node-pose", "left");
            } else if(m_limelight.getX() > 0) { //otherwise if the target is to the right
                SmartDashboard.putString("node-pose", "right");
            }
            target_offset = m_limelight.getX();
            if(target_offset > 0.1) {
                offset += 0.1;
            } else if (target_offset < -0.1){
                offset -= 0.1;
            }
        } else {
            SmartDashboard.putString("node-pose", "none");
        }
        wheelStrafe = MAX_WHEEL_STRAFE * Math.sin(Math.PI * (offset/MAX_CAMERA_X + 1));

        // if acquiring note, move forward to "chase" it
        // if aligning to april tag, don't move forward, just strafe
        //var forwardDistance = .85;//m_target == 0 ? .85 : 0;
        //m_drivetrain.moveRelative(forwardDistance, wheelStrafe, 0);
        m_drivetrain.moveRelative(0, wheelStrafe, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // use time as failsafe
        var area = m_limelight.getArea();
        var offset = Math.abs(m_limelight.getX());
        return t.hasElapsed(5) || (area > 0.05 && offset < 2);
        // return (area > 0.05 && offset < 2);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
    }
}
