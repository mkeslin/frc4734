// package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Commands.CenterToTargetCommand;
// import frc.robot.PathPlanner.PathPlanner;
// import frc.robot.Subsystems.Cameras.Limelight;

// public class LimelightAligner extends SubsystemBase {
    
//     public Limelight m_shooterLimelight;
//     public Limelight m_intakeLimelight;
//     public PathPlanner m_PathPlanner;

//     private CenterToTargetCommand m_centerIntakeCommand;
//     private CenterToTargetCommand m_centerShooterCommand;

//     private double MAX_WHEEL_STRAFE = 1;
//     private double MAX_CAMERA_X = 30;
//     private double m_wheelStrafe;

//     public LimelightAligner(Limelight shooterCamera, Limelight intakeCamera, PathPlanner pathPlanner) {
//         m_intakeLimelight = intakeCamera;
//         m_shooterLimelight = shooterCamera;
//         m_PathPlanner = pathPlanner;
//         m_centerIntakeCommand = new CenterToTargetCommand(m_intakeLimelight, m_PathPlanner, 0);
//         m_centerShooterCommand = new CenterToTargetCommand(m_shooterLimelight, m_PathPlanner, 0);
//     }

//     public Command alignToTag(int aprilTagId) {
//         return Commands.runOnce(() -> {centerToTarget(m_shooterLimelight, aprilTagId);});
//     }

//     public Command alignToNote() {
//         return Commands.runOnce(() -> {
//             SmartDashboard.putNumber("bruh", m_intakeLimelight.getX());
//             m_centerIntakeCommand.execute();
//             //centerToTarget(intakeLimelight, 0);
//         });
//     }

//     public void centerToTarget(Limelight limelight, int target) {
//         /*if the notes are being tracked and the target area is noticeable (not just a blip),
//         or if the AprilTags are being tracked and target is noticeable and the ID is correct*/
//         if(limelight.getArea() > 0.05 && (target < 1 || limelight.getAprilTagID() == target)) {
//             //if the target is to the left of the camera's sensor
//             if(limelight.getX() < 0) {
//                 SmartDashboard.putString("node-pose", "left");
//             } else if(limelight.getX() > 0) { //otherwise if the target is to the right
                
//                 SmartDashboard.putString("node-pose", "right");
//             }
//             m_wheelStrafe = MAX_WHEEL_STRAFE * Math.sin(Math.PI * (limelight.getX()/MAX_CAMERA_X + 1));
//             m_PathPlanner.moveRelative(1, m_wheelStrafe, 0);

//         } else {
//             SmartDashboard.putString("node-pose", "none");
//         }
//     }
// }
