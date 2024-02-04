package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;

public class LimelightAligner extends SubsystemBase {
    
    public Limelight shooterLimelight;
    public Limelight intakeLimelight;
    public PathPlanner m_PathPlanner;

    private CenterToTargetCommand centerIntakeCommand;
    private CenterToTargetCommand centerShooterCommand;

    private double MAX_WHEEL_STRAFE = 1;
    private double MAX_CAMERA_X = 30;
    private double wheelStrafe;

    public LimelightAligner(Limelight shooterCamera, Limelight intakeCamera, PathPlanner pathPlanner) {
        intakeLimelight = intakeCamera;
        shooterLimelight = shooterCamera;
        m_PathPlanner = pathPlanner;
        centerIntakeCommand = new CenterToTargetCommand(intakeLimelight, m_PathPlanner, 0);
        centerShooterCommand = new CenterToTargetCommand(shooterLimelight, m_PathPlanner, 0);
    }

    public Command alignToTag(int aprilTagId) {
        return Commands.runOnce(() -> {centerToTarget(shooterLimelight, aprilTagId);});
    }

    public Command alignToNote() {
        return Commands.runOnce(() -> {
            SmartDashboard.putNumber("bruh", intakeLimelight.getX());
            centerIntakeCommand.execute();
            //centerToTarget(intakeLimelight, 0);
        });
    }

    public void centerToTarget(Limelight limelight, int target) {
        /*if the notes are being tracked and the target area is noticeable (not just a blip),
        or if the AprilTags are being tracked and target is noticeable and the ID is correct*/
        if(limelight.getArea() > 0.05 && (target < 1 || limelight.getAprilTagID() == target)) {
            //if the target is to the left of the camera's sensor
            if(limelight.getX() < 0) {
                SmartDashboard.putString("node-pose", "left");
            } else if(limelight.getX() > 0) { //otherwise if the target is to the right
                
                SmartDashboard.putString("node-pose", "right");
            }
            wheelStrafe = MAX_WHEEL_STRAFE * Math.sin(Math.PI * (limelight.getX()/MAX_CAMERA_X + 1));
            m_PathPlanner.moveRelative(1, wheelStrafe, 0);

        } else {
            SmartDashboard.putString("node-pose", "none");
        }
    }
}
