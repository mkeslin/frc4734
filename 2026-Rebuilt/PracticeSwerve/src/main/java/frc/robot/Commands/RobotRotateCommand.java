// package frc.robot.Commands;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
// import frc.robot.SwerveDrivetrain.DrivetrainConstants;

// public class RobotRotateCommand extends Command {

//     private CommandSwerveDrivetrain m_drivetrain;

//     public double target_val;
//     public double current_val;
//     public boolean m_rotateCCW;

//     public Timer t = new Timer();

//     private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop

//     public RobotRotateCommand(CommandSwerveDrivetrain drivetrain, int degrees) {
//         m_drivetrain = drivetrain;
//         target_val = degrees;

//         // set target and noralize
//         target_val = normalizeAngle(target_val);

//         // get current and normalize
//         var currentPose = m_drivetrain.getPose();
//         var currentDegrees = currentPose.getRotation().getDegrees();
//         current_val = normalizeAngle(currentDegrees);

//         // get rotation direction
//         var normalizedDelta = normalizeAngle(target_val - current_val);
//         m_rotateCCW = normalizedDelta <= 180;

//         addRequirements(drivetrain);
//     }

//     private double normalizeAngle(double degrees) {
//         var normalizedAngle = degrees % 360;
//         if (normalizedAngle < 0) {
//             normalizedAngle += 360;
//         }
//         return normalizedAngle;
//     }

//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {
//         t.start();
//     }

//     @Override
//     public void execute() {
//         var direction = m_rotateCCW ? 1 : -1;
//         m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(direction * DrivetrainConstants.MaxAngularRate));
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         if (t.hasElapsed(4)) {
//             return true;
//         }

//         var currentPose = m_drivetrain.getPose();
//         var currentDegrees = currentPose.getRotation().getDegrees();
//         current_val = normalizeAngle(currentDegrees);

//         // Commands.print("target: " + target_val).schedule();
//         // Commands.print("current: " + current_val).schedule();

//         // var isFinished = (current_val <= target_val + 4) && (current_val >= target_val - 4);

//         var isFinished = false;
//         if (m_rotateCCW) {
//             isFinished = current_val >= target_val;
//         } else {
//             isFinished = current_val <= target_val;
//         }

//         // Commands.print("current2: " + current_val).schedule();
//         // Commands.print("is finished: " + isFinished).schedule();

//         return isFinished;
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean interrupted) {
//         m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

//         t.stop();
//         t.reset();
//     }
// }
