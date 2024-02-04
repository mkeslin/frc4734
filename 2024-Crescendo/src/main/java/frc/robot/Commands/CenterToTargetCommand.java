package frc.robot.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.AutoState;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LimelightAligner;
import frc.robot.Subsystems.Cameras.Limelight;

public class CenterToTargetCommand extends Command {
    public Limelight m_limelight;
    public PathPlanner m_PathPlanner;
    public int m_target;

    private double MAX_WHEEL_STRAFE = 1;
    private double MAX_CAMERA_X = 30;
    private double wheelStrafe;

    public CenterToTargetCommand(Limelight limelight, PathPlanner pathPlanner, int target) {
        m_limelight = limelight;
        m_PathPlanner = pathPlanner;
        m_target = target; //target should be 0 for Note alignment

        addRequirements(m_limelight, m_PathPlanner);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /*if the notes are being tracked and the target area is noticeable (not just a blip),
        or if the AprilTags are being tracked and target is noticeable and the ID is correct*/
        if(m_limelight.getArea() > 0.05 && (m_target < 1 || m_limelight.getAprilTagID() == m_target)) {
            //if the target is to the left of the camera's sensor
            if(m_limelight.getX() < 0) {
                SmartDashboard.putString("node-pose", "left");
            } else if(m_limelight.getX() > 0) { //otherwise if the target is to the right
                SmartDashboard.putString("node-pose", "right");
            }
            wheelStrafe = MAX_WHEEL_STRAFE * Math.sin(Math.PI * (m_limelight.getX()/MAX_CAMERA_X + 1));
            m_PathPlanner.moveRelative(1, wheelStrafe, 0);

        } else {
            SmartDashboard.putString("node-pose", "none");
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return m_limelight.getArea() > 0.05 && Math.abs(m_limelight.getX()) < 1;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {}
}
