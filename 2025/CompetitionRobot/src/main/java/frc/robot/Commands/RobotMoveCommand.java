// package frc.robot.Commands;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.PathPlanner.PathPlanner;
// import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

// public class RobotMoveCommand extends Command {

//     private CommandSwerveDrivetrain m_drivetrain;
//     private PathPlanner m_pathPlanner;

//     public Pose2d m_target_pose;

//     public Timer t = new Timer();

//     private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop

//     public RobotMoveCommand(CommandSwerveDrivetrain drivetrain, PathPlanner pathPlanner, Pose2d newPose) {
//         m_drivetrain = drivetrain;
//         m_pathPlanner = pathPlanner;

//         m_target_pose = newPose;

//         addRequirements(drivetrain, pathPlanner);
//     }

//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {
//         m_pathPlanner.moveToPose(m_target_pose).schedule();

//         t.start();
//     }

//     @Override
//     public void execute() {}

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         // finish if max duration elapsed
//         if (t.hasElapsed(4.5)) {
//             return true;
//         }

//         var currentPose = m_drivetrain.getPose();
//         var currentX = currentPose.getX();
//         var currentY = currentPose.getY();

//         var targetX = m_target_pose.getX();
//         var targetY = m_target_pose.getY();

//         var xIsFinished = (currentX <= targetX + .06) && (currentX >= targetX - .06);
//         var yIsFinished = (currentY <= targetY + .06) && (currentY >= targetY - .06);

//         // or if we are close enough to our target
//         if (xIsFinished && yIsFinished) {
//             return true;
//         }

//         // or if our speed drops below threshold
//         // var speeds = m_drivetrain.getRobotRelativeSpeeds();
//         // var xIsStopped = speeds.vxMetersPerSecond < .01;
//         // var yIsStopped = speeds.vyMetersPerSecond < .01;
//         // if (xIsStopped && yIsStopped) {
//         //     return true;
//         // }

//         // Commands.print("current2: " + current_val).schedule();
//         // Commands.print("is finished: " + isFinished).schedule();
//         return false;
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean interrupted) {
//         // stop robot
//         m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

//         t.stop();
//         t.reset();
//     }
// }
