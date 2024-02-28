package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.SwerveDrivetrain.DrivetrainConstants;

public class RobotRotateCommand extends Command {

    private CommandSwerveDrivetrain m_drivetrain;

    public double target_val;
    public double current_val;

    public Timer t = new Timer();

    private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop

    public RobotRotateCommand(CommandSwerveDrivetrain drivetrain, int degrees) {
        m_drivetrain = drivetrain;
        target_val = degrees;

        while (target_val < 0) {
            target_val += 360;
        }
        while (target_val >= 360) {
            target_val -= 360;
        }

        addRequirements(drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        var currentPose = m_drivetrain.getPose();
        var currentDegrees = currentPose.getRotation().getDegrees();
        current_val = currentDegrees % 360;

        t.start();
    }

    @Override
    public void execute() {
        m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(DrivetrainConstants.MaxAngularRate));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (t.hasElapsed(4.5)) {
            return true;
        }

        var currentPose = m_drivetrain.getPose();
        var current_val = currentPose.getRotation().getDegrees();

        while (current_val < 0) {
            current_val += 360;
        }
        while (current_val >= 360) {
            current_val -= 360;
        }

        // Commands.print("target: " + target_val).schedule();
        // Commands.print("current: " + current_val).schedule();

        var isFinished = (current_val <= target_val + 4) && (current_val >= target_val - 4);
        // Commands.print("current2: " + current_val).schedule();
        // Commands.print("is finished: " + isFinished).schedule();
        return isFinished;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

        t.reset();
    }
}
