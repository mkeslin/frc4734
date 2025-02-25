package frc.robot.Commands;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToReefCommand extends Command {
    public Limelight m_limelight;
    public CommandSwerveDrivetrain m_drivetrain;

    private double xSpeed, ySpeed, omegaSpeed;
    private int centerMethod;
    private int POSE = 0;
    private int CAMERA = 1;

    private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private HashMap<Integer, Pose2d> tagPoses = new HashMap<Integer, Pose2d>();

    private final PIDController xController = new PIDController(0.03, 0, 0);
    private final PIDController yController = new PIDController(0.03, 0, 0);
    private final PIDController omegaController = new PIDController(0.03, 0, 0);

    private double POSE_X_ERROR = 0.1;
    private double POSE_Y_ERROR = 0.1;
    private double POSE_ANGLE_ERROR = 3;

    private double CAMERA_AREA_GOAL = 40;
    private double CAMERA_AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 0.9;
    private double CAMERA_ANGLE_ERROR = 3;

    public Timer t = new Timer();

    public CenterToReefCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        centerMethod = POSE;

        for(int i = 6; i <= 11; i++) {
            tagPoses.put(Integer.valueOf(i), layout.getTagPose(i).get().toPose2d());
        }
        for(int i = 17; i <= 22; i++) {
            tagPoses.put(Integer.valueOf(i), layout.getTagPose(i).get().toPose2d());
        }
        
        setPIDValues(centerMethod);
        setTolerances(centerMethod);

        addRequirements(m_limelight, m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();
        setTargets(centerMethod);
    }

    @Override
    public void execute() {
        setSpeeds(centerMethod);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);

        m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return t.hasElapsed(5) || (centerMethod == CAMERA && !m_limelight.hasTargets()) || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()); //|| (area > FINAL_AREA && x_offset < FINAL_X_OFFSET && yaw_degrees < FINAL_ANGLE_DEGREES);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();
        m_drivetrain.setRelativeSpeed(0, 0, 0);
    }

    public Pose2d getClosestAprilTagTarget() {
        Integer closestTag = Integer.valueOf(0);
        double currentDistance;
        double closestDistance = Double.MAX_VALUE;
        Pose2d botPose = m_drivetrain.getPose();
        for (Integer i : tagPoses.keySet()) {
            currentDistance = Math.sqrt(Math.pow(tagPoses.get(i).getX() - botPose.getX(), 2) + Math.pow(tagPoses.get(i).getY() - botPose.getY(), 2));
            if(currentDistance < closestDistance) {
                closestTag = i;
                closestDistance = currentDistance;
            }
        }
        SmartDashboard.putNumber("target", closestTag);
        return tagPoses.get(closestTag);
    }

    public void setPIDValues(int method) {
        if(method == POSE) {
            xController.setP(0.3);
            yController.setP(0.3);
        } else if(method == CAMERA) {
            xController.setP(0.03);
            yController.setP(0.03);
        }
    }

    public void setTolerances(int method) {
        if(method == POSE) {
            xController.setTolerance(POSE_X_ERROR);
            yController.setTolerance(POSE_Y_ERROR);
            omegaController.setTolerance(POSE_ANGLE_ERROR);
        } else if(method == CAMERA) {
            xController.setTolerance(CAMERA_AREA_ERROR);
            yController.setTolerance(CAMERA_X_OFFSET_ERROR);
            omegaController.setTolerance(CAMERA_ANGLE_ERROR);
        }
    }

    public void setTargets(int method) {
        if(method == POSE) {
            Pose2d target = getClosestAprilTagTarget();
            double tagRot = target.getRotation().getDegrees();
            double targetRot = (tagRot < 180) ? tagRot + 180 : tagRot - 180;
            SmartDashboard.putNumber("xTarget", target.getX());
            SmartDashboard.putNumber("yTarget", target.getY());
            xController.setSetpoint(target.getX());
            yController.setSetpoint(target.getY());
            omegaController.setSetpoint(targetRot);
        } else if(method == CAMERA) {
            xController.setSetpoint(CAMERA_AREA_GOAL);
            yController.setSetpoint(0);
            omegaController.setSetpoint(0);
        }
    }

    public void setSpeeds(int method) {
        if(method == POSE) {
            Pose2d position = m_drivetrain.getPose();
            SmartDashboard.putNumber("xPos", position.getX());
            SmartDashboard.putNumber("yPos", position.getY());
            SmartDashboard.putNumber("omegaPos", position.getRotation().getDegrees());
            xSpeed = xController.calculate(position.getX());
            ySpeed = yController.calculate(position.getY());
            omegaSpeed = omegaController.calculate(position.getRotation().getDegrees());
        } else if(method == CAMERA) {
            xSpeed = xController.calculate(m_limelight.getArea());
            ySpeed = yController.calculate(m_limelight.getX());
            omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_limelight.getYaw()));
            if (!m_limelight.hasTargets()) {
                ySpeed = 0;
                xSpeed = 0;
                omegaSpeed = 0;
            }
        }
    }

    public void getSpeeds(double x, double y, double omega) {
        SmartDashboard.putNumber("xSpeed", x);
        SmartDashboard.putNumber("ySpeed", y);
        SmartDashboard.putNumber("omegaSpeed", omega);
    }
}