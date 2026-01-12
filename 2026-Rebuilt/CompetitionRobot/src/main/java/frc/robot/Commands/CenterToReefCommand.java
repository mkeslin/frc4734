package frc.robot.Commands;

import java.util.HashMap;
import java.util.Objects;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to center the robot to the reef scoring station using PhotonVision.
 * Supports two centering methods: camera-based (using area, X offset, and yaw) and
 * pose-based (using estimated robot pose from AprilTags). Uses PID controllers to
 * align the robot and completes when the target position is reached, vision is lost,
 * timeout occurs, or driver interrupts.
 * 
 * <p>This command pre-calculates target poses for AprilTags 6-11 and 17-22 during
 * construction, positioning the robot at a fixed distance from each tag.
 * 
 * @see PhotonVision
 * @see CommandSwerveDrivetrain
 */
public class CenterToReefCommand extends Command {
    public PhotonVision m_photonVision;
    public CommandSwerveDrivetrain m_drivetrain;
    public CommandXboxController m_driveController;

    private double xSpeed, ySpeed, omegaSpeed;
    private Pose2d tagPose, targetPose;
    private boolean driverInterrupted;
    private int centerMethod;
    private int POSE = 0;
    private int CAMERA = 1;

    private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private HashMap<Integer, Pose2d> tagPoses = new HashMap<Integer, Pose2d>();

    private final PIDController xController = new PIDController(0.03, 0, 0);
    private final PIDController yController = new PIDController(0.03, 0, 0);
    private final PIDController omegaController = new PIDController(0.03, 0, 0);

    // NOTE: Nick, please review
    private double CAMERA_Y_OFFSET = -6;

    private double DISTANCE_FROM_APRILTAG = 0.75;

    private double POSE_X_ERROR = 0.06;
    private double POSE_Y_ERROR = 0.05;
    private double POSE_ANGLE_ERROR = 3;

    private double CAMERA_AREA_GOAL = 37;
    private double CAMERA_AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 0.9;
    private double CAMERA_ANGLE_ERROR = 2.5;

    private double m_timeoutDuration;

    public Timer t = new Timer();

    /**
     * Creates a new CenterToReefCommand.
     * 
     * @param photonVision The PhotonVision camera subsystem for vision feedback
     * @param drivetrain The swerve drivetrain to control
     * @param driveController The Xbox controller for driver interruption (can be null)
     * @param timeoutDuration The maximum time in seconds before the command times out (must be > 0)
     * @throws NullPointerException if photonVision or drivetrain is null
     * @throws IllegalArgumentException if timeoutDuration is less than or equal to 0
     */
    public CenterToReefCommand(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain,
            CommandXboxController driveController, double timeoutDuration) {
        m_photonVision = Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
        m_drivetrain = Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        m_driveController = driveController; // Can be null (optional)
        
        if (timeoutDuration <= 0) {
            throw new IllegalArgumentException("Timeout duration must be greater than 0, got: " + timeoutDuration);
        }
        m_timeoutDuration = timeoutDuration;

        centerMethod = CAMERA;

        for (int i = 6; i <= 11; i++) {
            var tagPoseOptional = layout.getTagPose(i);
            if (tagPoseOptional.isEmpty()) {
                DataLogManager.log(String.format("[CenterToReef] WARN: AprilTag %d not found in layout, skipping", i));
                continue;
            }
            tagPose = tagPoseOptional.get().toPose2d();
            targetPose = new Pose2d(tagPose.getTranslation()
                    .plus(new Translation2d(tagPose.getRotation().getCos() * DISTANCE_FROM_APRILTAG,
                            tagPose.getRotation().getSin() * DISTANCE_FROM_APRILTAG)),
                    tagPose.getRotation());
            tagPoses.put(Integer.valueOf(i), targetPose);
        }
        for (int i = 17; i <= 22; i++) {
            var tagPoseOptional = layout.getTagPose(i);
            if (tagPoseOptional.isEmpty()) {
                DataLogManager.log(String.format("[CenterToReef] WARN: AprilTag %d not found in layout, skipping", i));
                continue;
            }
            tagPose = tagPoseOptional.get().toPose2d();
            targetPose = new Pose2d(tagPose.getTranslation()
                    .plus(new Translation2d(tagPose.getRotation().getCos() * DISTANCE_FROM_APRILTAG,
                            tagPose.getRotation().getSin() * DISTANCE_FROM_APRILTAG)),
                    tagPose.getRotation());
            tagPoses.put(Integer.valueOf(i), targetPose);
        }

        setPIDValues(centerMethod);
        setTolerances(centerMethod);

        addRequirements(m_photonVision, m_drivetrain);
    }

    /**
     * Initializes the command by starting the timer, setting targets based on the centering method,
     * and resetting the driver interruption flag.
     */
    @Override
    public void initialize() {
        t.start();
        setTargets(centerMethod);
        m_drivetrain.setRelativeSpeed(0, 0, 0);
        driverInterrupted = false;
    }

    /**
     * Executes the command by calculating speeds based on the selected centering method
     * and applying them to the drivetrain. Publishes speeds to SmartDashboard for debugging.
     */
    @Override
    public void execute() {
        setSpeeds(centerMethod);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);

        m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
        if (m_driveController != null) {
            m_driveController.povUp().onTrue(Commands.runOnce(() -> {
                driverInterrupted = true;
            }));
        }
    }

    private boolean hasPrintedX = false;
    private boolean hasPrintedY = false;
    private boolean hasPrintedZ = false;

    /**
     * Determines if the command should finish. Logs when each axis reaches its setpoint.
     * 
     * @return true if the timeout has elapsed, driver interrupted, no targets detected,
     *         or all PID controllers are at their setpoints
     */
    @Override
    public boolean isFinished() {
        if (!hasPrintedX && xController.atSetpoint()) {
            DataLogManager.log("[CenterToReef] X axis at setpoint");
            hasPrintedX = true;
        }
        if (!hasPrintedY && yController.atSetpoint()) {
            DataLogManager.log("[CenterToReef] Y axis at setpoint");
            hasPrintedY = true;
        }
        if (!hasPrintedZ && omegaController.atSetpoint()) {
            DataLogManager.log("[CenterToReef] Omega (rotation) axis at setpoint");
            hasPrintedZ = true;
        }
        
        if (t.hasElapsed(m_timeoutDuration)) {
            DataLogManager.log(String.format("[CenterToReef] WARN: Command timed out after %.2f seconds", m_timeoutDuration));
            return true;
        }
        if (driverInterrupted) {
            DataLogManager.log("[CenterToReef] Command interrupted by driver");
            return true;
        }
        if (centerMethod == CAMERA && !m_photonVision.hasTargets()) {
            DataLogManager.log("[CenterToReef] WARN: Lost AprilTag target");
            return true;
        }
        if (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()) {
            DataLogManager.log("[CenterToReef] Successfully centered to reef");
            return true;
        }

        return false;
    }

    /**
     * Called when the command ends. Stops and resets the timer, and stops drivetrain movement.
     * 
     * @param interrupted true if the command was interrupted, false if it completed normally
     */
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
            currentDistance = Math.sqrt(Math.pow(tagPoses.get(i).getX() - botPose.getX(), 2)
                    + Math.pow(tagPoses.get(i).getY() - botPose.getY(), 2));
            if (currentDistance < closestDistance) {
                closestTag = i;
                closestDistance = currentDistance;
            }
        }
        SmartDashboard.putNumber("target", closestTag);
        return tagPoses.get(closestTag);
    }

    public void setPIDValues(int method) {
        if (method == POSE) {
            xController.setP(1);
            yController.setP(1);
        } else if (method == CAMERA) {
            xController.setP(0.03);
            yController.setP(0.03);
        }
    }

    public void setTolerances(int method) {
        if (method == POSE) {
            xController.setTolerance(POSE_X_ERROR);
            yController.setTolerance(POSE_Y_ERROR);
            omegaController.setTolerance(POSE_ANGLE_ERROR);
        } else if (method == CAMERA) {
            xController.setTolerance(CAMERA_AREA_ERROR);
            yController.setTolerance(CAMERA_X_OFFSET_ERROR);
            omegaController.setTolerance(CAMERA_ANGLE_ERROR);
        }
    }

    public void setTargets(int method) {
        if (method == POSE) {
            Pose2d target = getClosestAprilTagTarget();
            double tagRot = target.getRotation().getDegrees();
            double targetRot = (tagRot < 0) ? tagRot + 180 : tagRot - 180;
            SmartDashboard.putNumber("xTarget", target.getX());
            SmartDashboard.putNumber("yTarget", target.getY());
            SmartDashboard.putNumber("omegaTarget", targetRot);
            xController.setSetpoint(target.getX());
            yController.setSetpoint(target.getY());
            omegaController.setSetpoint(targetRot);
        } else if (method == CAMERA) {
            xController.setSetpoint(CAMERA_AREA_GOAL);
            yController.setSetpoint(CAMERA_Y_OFFSET);
            omegaController.setSetpoint(0);
        }
    }

    public void setSpeeds(int method) {
        if (method == POSE) {
            Pose2d position = m_drivetrain.getPose();
            SmartDashboard.putNumber("xPos", position.getX());
            SmartDashboard.putNumber("yPos", position.getY());
            SmartDashboard.putNumber("omegaPos", position.getRotation().getDegrees());
            if (Math.abs(omegaController.getSetpoint()) == 180
                    && Math.signum(omegaController.getSetpoint()) != Math.signum(position.getRotation().getDegrees())) {
                omegaController.setSetpoint(-omegaController.getSetpoint());
            }
            xSpeed = 2 * Math.signum(position.getRotation().getCos()) * xController.calculate(position.getX());
            ySpeed = 2 * Math.signum(position.getRotation().getCos()) * yController.calculate(position.getY());
            omegaSpeed = omegaController.calculate(position.getRotation().getDegrees());
        } else if (method == CAMERA) {
            xSpeed = xController.calculate(m_photonVision.getArea());
            ySpeed = yController.calculate(m_photonVision.getX());
            omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_photonVision.getYaw()));
            if (!m_photonVision.hasTargets()) {
                ySpeed = 0;
                xSpeed = 0;
                omegaSpeed = 0;
            }
        }
    }

    public void getSpeeds() {
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);
    }
}