package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToTargetCommand extends Command {
    public Limelight m_limelight;
    public CommandSwerveDrivetrain m_drivetrain;

    //private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0.2, 0.1);
    //private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
    //private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 4);

    //private static final int TAG_TO_CHASE = 2;
    /*private static final Transform3d TAG_TO_GOAL = new Transform3d(
        new Translation3d(1.5, 0, 0),
        new Rotation3d(0, 0, Math.PI)
    );*/
    //private final Supplier<Pose2d> poseProvider;

    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController yController = new PIDController(0.05, 0, 0);
    private final PIDController omegaController = new PIDController(0.05, 0, 0);

    /*private double MAX_WHEEL_STRAFE = 0.75;
    private double MAX_WHEEL_ROTATE = 0.5;
    private double MAX_CAMERA_X = 30;*/

    private double FINAL_AREA = 10;
    private double AREA_ERROR = 2;
    private double X_OFFSET_ERROR = 2;
    private double ANGLE_ERROR = 5;

    /*private double target_x_offset;
    private double target_yaw;
    private double target_area;

    private boolean rotAligned = false;*/

    public Timer t = new Timer();

    public CenterToTargetCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;

        xController.setTolerance(AREA_ERROR);
        yController.setTolerance(X_OFFSET_ERROR);
        omegaController.setTolerance(ANGLE_ERROR);
        //omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_limelight, m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();

        xController.setSetpoint(FINAL_AREA);
        yController.setSetpoint(0);
        omegaController.setSetpoint(0);
        
        //var robotPose = poseProvider.get();
        //omegaController.reset(robotPose.getRotation().getRadians());
        //xController.reset(robotPose.getX());
        //yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {

        //var robotPose2D = poseProvider.get();
        //var robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0, new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));
        if(m_limelight.hasTargets()) {
            //possibly add additional conditions for specific target IDs
            
            var xSpeed = xController.calculate(m_limelight.getArea());
            if (xController.atSetpoint()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(m_limelight.getX());
            if (yController.atSetpoint()) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_limelight.getYaw()));
            if (omegaController.atSetpoint()) {
                omegaSpeed = 0;
            }
            m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
        }
        /*target_area = m_limelight.getArea();
        target_yaw = m_limelight.getYaw();
        target_x_offset = m_limelight.getX();
        //wheelRotate = MAX_WHEEL_ROTATE * Math.sin(Math.PI * (target_yaw * 2/Math.PI));
        var forwardDistance = (target_area <= FINAL_AREA && target_area > 0) ? 0.5 : 0;
        var strafeDistance = MAX_WHEEL_STRAFE * Math.sin(Math.PI * (target_x_offset/MAX_CAMERA_X + 1));
        var rotateDistance = (Math.abs(target_yaw * 180/Math.PI) >= FINAL_ANGLE_DEGREES) ? MAX_WHEEL_ROTATE * -Math.signum(target_yaw) : 0;
        if(!rotAligned) {
            m_drivetrain.moveRelative(0, 0, rotateDistance);
            if(Math.abs(target_yaw * 180/Math.PI) < FINAL_ANGLE_DEGREES) {
                rotAligned = true;
            }
        } else {
            SmartDashboard.putNumber("forward", forwardDistance);
            SmartDashboard.putNumber("strafe", strafeDistance);
            SmartDashboard.putNumber("rotate", rotateDistance);
            m_drivetrain.moveRelative(forwardDistance, strafeDistance, rotateDistance);
        }*/
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        //var area = m_limelight.getArea();
        //var x_offset = Math.abs(m_limelight.getX());
        //var yaw_degrees = Math.abs(m_limelight.getYaw() * 180/Math.PI);
        // use time as failsafe
        return t.hasElapsed(5) || !m_limelight.hasTargets() || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()); //|| (area > FINAL_AREA && x_offset < FINAL_X_OFFSET && yaw_degrees < FINAL_ANGLE_DEGREES);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
    }
}
