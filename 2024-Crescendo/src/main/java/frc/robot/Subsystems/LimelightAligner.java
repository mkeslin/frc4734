package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;

public class LimelightAligner extends SubsystemBase {
    
    public Limelight shooterLimelight;
    public Limelight intakeLimelight;
    public PathPlanner m_PathPlanner;

    public LimelightAligner(Limelight shooterCamera, Limelight intakeCamera, PathPlanner pathPlanner) {
        intakeLimelight = intakeCamera;
        shooterLimelight = shooterCamera;
        m_PathPlanner = pathPlanner;
    }

    public Command alignToTag(int aprilTagId) {
        return Commands.runOnce(() -> {centerToTarget(shooterLimelight, aprilTagId);});
    }

    public Command alignToNote() {
        return Commands.run(() -> {
            SmartDashboard.putNumber("bruh", intakeLimelight.getX());
            centerToTarget(intakeLimelight, 0);
            m_PathPlanner.moveRelative(1, 0, 0);
        });
    }

    public void centerToTarget(Limelight limelight, int target) {
        /*if the notes are being tracked and the target area is noticeable (not just a blip),
        or if the AprilTags are being tracked and target is noticeable and the ID is correct*/
        if(limelight.getArea() > 0.05 && (target < 1 || limelight.getAprilTagID() == target)) {
            //if the target is to the left of the camera's sensor
            if(limelight.getX() < 0) {
                //m_PathPlanner.moveRelative(10, 0, 0);
                SmartDashboard.putString("node-pose", "left");
            } else if(limelight.getX() > 0) { //otherwise if the target is to the right
                //m_PathPlanner.moveRelative(10, 0, 0);
                SmartDashboard.putString("node-pose", "right");
            }
        } else {
            SmartDashboard.putString("node-pose", "none");
        }
    }
}
