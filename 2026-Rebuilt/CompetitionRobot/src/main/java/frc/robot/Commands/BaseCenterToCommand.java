package frc.robot.Commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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
 * Base command for centering the robot to targets using PhotonVision.
 * Supports two centering methods: camera-based (using area, X offset, and yaw) and
 * pose-based (using estimated robot pose from AprilTags). Uses PID controllers to
 * align the robot and completes when the target position is reached, vision is lost,
 * timeout occurs, or driver interrupts.
 * 
 * <p>This class provides a generic implementation that can be configured with
 * specific parameters for different centering scenarios. Subclasses can override
 * hook methods to add custom behavior.
 * 
 * @see PhotonVision
 * @see CommandSwerveDrivetrain
 */
public abstract class BaseCenterToCommand extends Command {
    /**
     * Enumeration of centering methods supported by the command.
     */
    public enum CenterMethod {
        /** Uses robot pose estimation to navigate to pre-calculated AprilTag positions */
        POSE,
        /** Uses PhotonVision camera feedback (area, X offset, yaw) */
        CAMERA
    }

    protected final PhotonVision m_photonVision;
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final CommandXboxController m_driveController;

    private double xSpeed, ySpeed, omegaSpeed;
    private boolean driverInterrupted;
    private final CenterMethod centerMethod;

    private final AprilTagFieldLayout layout;
    private final HashMap<Integer, Pose2d> tagPoses = new HashMap<Integer, Pose2d>();

    protected final PIDController xController;
    protected final PIDController yController;
    protected final PIDController omegaController;

    // Camera method parameters
    private final double cameraAreaGoal;
    private final double cameraYOffset;
    private final double cameraAreaError;
    private final double cameraXOffsetError;
    private final double cameraAngleError;
    private final double cameraXP;
    private final double cameraYP;
    private final double cameraOmegaP;
    private final boolean negateCameraSpeeds;

    // Pose method parameters
    private final double distanceFromAprilTag;
    private final double poseXError;
    private final double poseYError;
    private final double poseAngleError;
    private final double poseXP;
    private final double poseYP;
    private final double poseOmegaP;

    private final double timeoutDuration;
    private final List<Integer> tagNumbers;

    protected final Timer t = new Timer();

    private boolean hasPrintedX = false;
    private boolean hasPrintedY = false;
    private boolean hasPrintedZ = false;

    /**
     * Configuration class for BaseCenterToCommand parameters.
     * Use this to build a configuration and pass it to the constructor.
     */
    public static class Config {
        // Required
        public PhotonVision photonVision;
        public CommandSwerveDrivetrain drivetrain;
        public CenterMethod centerMethod;
        public double timeoutDuration;

        // Optional - defaults provided
        public CommandXboxController driveController = null;
        public List<Integer> tagNumbers = new ArrayList<>();
        public double distanceFromAprilTag = 0.75;

        // Camera method defaults
        public double cameraAreaGoal = 37.0;
        public double cameraYOffset = -6.0;
        public double cameraAreaError = 2.0;
        public double cameraXOffsetError = 0.9;
        public double cameraAngleError = 2.5;
        public double cameraXP = 0.03;
        public double cameraYP = 0.03;
        public double cameraOmegaP = 0.03;
        public boolean negateCameraSpeeds = false; // Set to true to negate x and y speeds for camera method

        // Pose method defaults
        public double poseXError = 0.06;
        public double poseYError = 0.05;
        public double poseAngleError = 3.0;
        public double poseXP = 1.0;
        public double poseYP = 1.0;
        public double poseOmegaP = 0.03;

        /**
         * Creates a new configuration with required parameters.
         * 
         * @param photonVision The PhotonVision camera subsystem
         * @param drivetrain The swerve drivetrain to control
         * @param centerMethod The centering method to use
         * @param timeoutDuration The timeout duration in seconds (must be > 0)
         */
        public Config(PhotonVision photonVision, CommandSwerveDrivetrain drivetrain,
                CenterMethod centerMethod, double timeoutDuration) {
            this.photonVision = Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
            this.drivetrain = Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
            this.centerMethod = Objects.requireNonNull(centerMethod, "CenterMethod cannot be null");
            if (timeoutDuration <= 0) {
                throw new IllegalArgumentException("Timeout duration must be greater than 0, got: " + timeoutDuration);
            }
            this.timeoutDuration = timeoutDuration;
        }

        /**
         * Adds a range of tag numbers for pose method.
         * 
         * @param startTag First tag in range (inclusive)
         * @param endTag Last tag in range (inclusive)
         * @return This config for method chaining
         */
        public Config addTagRange(int startTag, int endTag) {
            for (int i = startTag; i <= endTag; i++) {
                tagNumbers.add(i);
            }
            return this;
        }

        /**
         * Adds a single tag number for pose method.
         * 
         * @param tagNumber The tag number to add
         * @return This config for method chaining
         */
        public Config addTag(int tagNumber) {
            tagNumbers.add(tagNumber);
            return this;
        }
    }

    /**
     * Creates a new BaseCenterToCommand with the specified configuration.
     * 
     * @param config The configuration object containing all parameters
     * @throws NullPointerException if required parameters in config are null
     * @throws IllegalArgumentException if timeoutDuration is less than or equal to 0
     */
    protected BaseCenterToCommand(Config config) {
        m_photonVision = config.photonVision;
        m_drivetrain = config.drivetrain;
        m_driveController = config.driveController;
        centerMethod = config.centerMethod;
        timeoutDuration = config.timeoutDuration;
        tagNumbers = new ArrayList<>(config.tagNumbers);

        // Camera method parameters
        cameraAreaGoal = config.cameraAreaGoal;
        cameraYOffset = config.cameraYOffset;
        cameraAreaError = config.cameraAreaError;
        cameraXOffsetError = config.cameraXOffsetError;
        cameraAngleError = config.cameraAngleError;
        cameraXP = config.cameraXP;
        cameraYP = config.cameraYP;
        cameraOmegaP = config.cameraOmegaP;
        negateCameraSpeeds = config.negateCameraSpeeds;

        // Pose method parameters
        distanceFromAprilTag = config.distanceFromAprilTag;
        poseXError = config.poseXError;
        poseYError = config.poseYError;
        poseAngleError = config.poseAngleError;
        poseXP = config.poseXP;
        poseYP = config.poseYP;
        poseOmegaP = config.poseOmegaP;

        // Initialize PID controllers with default values (will be set based on method)
        xController = new PIDController(0.03, 0, 0);
        yController = new PIDController(0.03, 0, 0);
        omegaController = new PIDController(0.03, 0, 0);

        // Load field layout and pre-calculate tag poses if using POSE method
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        if (centerMethod == CenterMethod.POSE && !tagNumbers.isEmpty()) {
            calculateTagPoses();
        }

        // Set PID values and tolerances based on method
        setPIDValues(centerMethod);
        setTolerances(centerMethod);

        addRequirements(m_photonVision, m_drivetrain);
    }

    /**
     * Pre-calculates target poses for all configured AprilTags.
     * Positions the robot at the specified distance from each tag.
     */
    private void calculateTagPoses() {
        for (Integer tagId : tagNumbers) {
            var tagPoseOptional = layout.getTagPose(tagId);
            if (tagPoseOptional.isEmpty()) {
                DataLogManager.log(String.format("[BaseCenterTo] WARN: AprilTag %d not found in layout, skipping", tagId));
                continue;
            }
            Pose2d tagPose = tagPoseOptional.get().toPose2d();
            Pose2d targetPose = new Pose2d(
                    tagPose.getTranslation()
                            .plus(new Translation2d(
                                    tagPose.getRotation().getCos() * distanceFromAprilTag,
                                    tagPose.getRotation().getSin() * distanceFromAprilTag)),
                    tagPose.getRotation());
            tagPoses.put(tagId, targetPose);
        }
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
        hasPrintedX = false;
        hasPrintedY = false;
        hasPrintedZ = false;
        onInitialize();
    }

    /**
     * Hook method called during initialization. Override to add custom initialization logic.
     */
    protected void onInitialize() {
        // Default implementation does nothing
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
        onExecute();
    }

    /**
     * Hook method called during execution. Override to add custom execution logic.
     */
    protected void onExecute() {
        // Default implementation does nothing
    }

    /**
     * Determines if the command should finish. Logs when each axis reaches its setpoint.
     * 
     * @return true if the timeout has elapsed, driver interrupted, no targets detected,
     *         or all PID controllers are at their setpoints
     */
    @Override
    public boolean isFinished() {
        // Log setpoint achievements
        if (!hasPrintedX && xController.atSetpoint()) {
            DataLogManager.log(getCommandName() + " X axis at setpoint");
            hasPrintedX = true;
        }
        if (!hasPrintedY && yController.atSetpoint()) {
            DataLogManager.log(getCommandName() + " Y axis at setpoint");
            hasPrintedY = true;
        }
        if (!hasPrintedZ && omegaController.atSetpoint()) {
            DataLogManager.log(getCommandName() + " Omega (rotation) axis at setpoint");
            hasPrintedZ = true;
        }

        // Check timeout
        if (t.hasElapsed(timeoutDuration)) {
            DataLogManager.log(String.format("[%s] WARN: Command timed out after %.2f seconds", getCommandName(), timeoutDuration));
            return true;
        }

        // Check driver interruption
        if (driverInterrupted) {
            DataLogManager.log(getCommandName() + " Command interrupted by driver");
            return true;
        }

        // Check vision targets (for camera method)
        if (centerMethod == CenterMethod.CAMERA && !m_photonVision.hasTargets()) {
            DataLogManager.log(getCommandName() + " WARN: Lost AprilTag target");
            return true;
        }

        // Check additional finish conditions (hook method)
        if (additionalFinishConditions()) {
            return true;
        }

        // Check if all PID controllers are at setpoint
        if (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()) {
            DataLogManager.log(getCommandName() + " Successfully centered");
            return true;
        }

        return false;
    }

    /**
     * Hook method for additional finish conditions. Override to add custom finish logic.
     * 
     * @return true if the command should finish due to additional conditions
     */
    protected boolean additionalFinishConditions() {
        return false;
    }

    /**
     * Gets the command name for logging purposes. Override to provide a custom name.
     * 
     * @return The command name
     */
    protected String getCommandName() {
        return "[BaseCenterTo]";
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
        if (interrupted) {
            onInterrupted();
        } else {
            onFinished();
        }
    }

    /**
     * Hook method called when the command finishes successfully.
     * Override to add custom completion logic.
     */
    protected void onFinished() {
        // Default implementation does nothing
    }

    /**
     * Hook method called when the command is interrupted.
     * Override to add custom interruption logic.
     */
    protected void onInterrupted() {
        // Default implementation does nothing
    }

    /**
     * Gets the closest AprilTag target pose based on current robot position.
     * Only valid when using POSE method.
     * 
     * @return The closest target pose
     */
    protected Pose2d getClosestAprilTagTarget() {
        Integer closestTag = Integer.valueOf(0);
        double currentDistance;
        double closestDistance = Double.MAX_VALUE;
        Pose2d botPose = m_drivetrain.getPose();
        for (Integer i : tagPoses.keySet()) {
            currentDistance = Math.sqrt(
                    Math.pow(tagPoses.get(i).getX() - botPose.getX(), 2)
                            + Math.pow(tagPoses.get(i).getY() - botPose.getY(), 2));
            if (currentDistance < closestDistance) {
                closestTag = i;
                closestDistance = currentDistance;
            }
        }
        SmartDashboard.putNumber("target", closestTag);
        return tagPoses.get(closestTag);
    }

    /**
     * Sets PID controller values based on the centering method.
     * 
     * @param method The centering method to use
     */
    private void setPIDValues(CenterMethod method) {
        if (method == CenterMethod.POSE) {
            xController.setP(poseXP);
            yController.setP(poseYP);
            omegaController.setP(poseOmegaP);
        } else if (method == CenterMethod.CAMERA) {
            xController.setP(cameraXP);
            yController.setP(cameraYP);
            omegaController.setP(cameraOmegaP);
        }
    }

    /**
     * Sets PID controller tolerances based on the centering method.
     * 
     * @param method The centering method to use
     */
    private void setTolerances(CenterMethod method) {
        if (method == CenterMethod.POSE) {
            xController.setTolerance(poseXError);
            yController.setTolerance(poseYError);
            omegaController.setTolerance(poseAngleError);
        } else if (method == CenterMethod.CAMERA) {
            xController.setTolerance(cameraAreaError);
            yController.setTolerance(cameraXOffsetError);
            omegaController.setTolerance(cameraAngleError);
        }
    }

    /**
     * Sets PID controller setpoints based on the centering method.
     * 
     * @param method The centering method to use
     */
    private void setTargets(CenterMethod method) {
        if (method == CenterMethod.POSE) {
            Pose2d target = getClosestAprilTagTarget();
            double tagRot = target.getRotation().getDegrees();
            double targetRot = (tagRot < 0) ? tagRot + 180 : tagRot - 180;
            SmartDashboard.putNumber("xTarget", target.getX());
            SmartDashboard.putNumber("yTarget", target.getY());
            SmartDashboard.putNumber("omegaTarget", targetRot);
            xController.setSetpoint(target.getX());
            yController.setSetpoint(target.getY());
            omegaController.setSetpoint(targetRot);
        } else if (method == CenterMethod.CAMERA) {
            xController.setSetpoint(cameraAreaGoal);
            yController.setSetpoint(cameraYOffset);
            omegaController.setSetpoint(0);
        }
    }

    /**
     * Calculates and sets the speeds based on the centering method.
     * 
     * @param method The centering method to use
     */
    private void setSpeeds(CenterMethod method) {
        if (method == CenterMethod.POSE) {
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
        } else if (method == CenterMethod.CAMERA) {
            double calculatedXSpeed = xController.calculate(m_photonVision.getArea());
            double calculatedYSpeed = yController.calculate(m_photonVision.getX());
            omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_photonVision.getYaw()));
            if (!m_photonVision.hasTargets()) {
                ySpeed = 0;
                xSpeed = 0;
                omegaSpeed = 0;
            } else {
                xSpeed = negateCameraSpeeds ? -calculatedXSpeed : calculatedXSpeed;
                ySpeed = negateCameraSpeeds ? -calculatedYSpeed : calculatedYSpeed;
            }
        }
    }
}
