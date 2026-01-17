package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.AutoManager;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.RobotContext;
import frc.robot.Logging.RobotLogger;
import frc.robot.Monitoring.PerformanceMonitor;
import frc.robot.Controllers.ControllerIds;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Lights;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainA;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainBindings;

/**
 * Main container class for robot subsystems, controllers, and configuration.
 * Coordinates subsystem creation, controller bindings, and autonomous setup
 * using specialized factory and configurator classes.
 */
public class RobotContainer {

    // CONTROLLERS
    private final CommandXboxController m_driveController = new CommandXboxController(ControllerIds.XC1ID);
    private final CommandXboxController m_mechanismController = new CommandXboxController(ControllerIds.XC2ID);
    private final CommandXboxController m_arcadeController = new CommandXboxController(ControllerIds.XC3ID);

    // DRIVETRAIN
    public final CommandSwerveDrivetrain m_drivetrain = SwerveDrivetrainA.createDrivetrain();

    // AUTO MANAGER
    private final AutoManager m_autoManager;

    // FACTORY AND CONFIGURATORS
    private final SubsystemFactory m_subsystemFactory;
    // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
    private final BindingConfigurator m_bindingConfigurator; // May be null during drivetrain-only testing
    private final AutoConfigurator m_autoConfigurator; // May be null during drivetrain-only testing

    // COMMANDS
    public final CenterToReefCommand m_centerToReefCommand; // May be null during drivetrain-only testing

    public RobotContainer() {
        // Create AutoManager instance
        m_autoManager = new AutoManager();
        m_autoManager.init();

        // Set AutoManager on drivetrain for odometry reset
        m_drivetrain.setAutoManager(m_autoManager);

        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // Create subsystem factory (creates all subsystems, PositionTracker, StateMachine, RobotContext)
        m_subsystemFactory = new SubsystemFactory(m_drivetrain);

        // Create center to reef command
        // m_centerToReefCommand = new CenterToReefCommand(
        //         m_subsystemFactory.getPhotonVision(),
        //         m_drivetrain,
        //         m_driveController,
        //         3);
        m_centerToReefCommand = null; // Temporary for drivetrain-only testing

        // Register named commands
        // NamedCommands.registerCommand("centerToReefCommand", m_centerToReefCommand);

        // Configure bindings for swerve drivetrain
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);

        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // Create binding configurator and configure all controller bindings
        // m_bindingConfigurator = new BindingConfigurator(
        //         m_driveController,
        //         m_mechanismController,
        //         m_arcadeController,
        //         m_subsystemFactory.getRobotContext(),
        //         m_subsystemFactory.getClimber(),
        //         m_centerToReefCommand);
        // m_bindingConfigurator.configureAllBindings();
        m_bindingConfigurator = null; // Temporary for drivetrain-only testing

        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // Create auto configurator and configure autonomous routines
        // m_autoConfigurator = new AutoConfigurator(
        //         m_autoManager,
        //         m_subsystemFactory.getRobotContext(),
        //         m_centerToReefCommand);
        // m_autoConfigurator.configureAuto();
        m_autoConfigurator = null; // Temporary for drivetrain-only testing

        // Seed field-centric pose
        m_drivetrain.seedFieldCentric();

        // Initialize subsystems
        CommandScheduler.getInstance().schedule(RobotState.getInstance().enableInitializationCommand());

        // Reset positions
        m_subsystemFactory.resetZeros();
    }

    /**
     * Localizes the robot pose using PhotonVision AprilTag detection.
     * Updates the drivetrain's pose estimator with vision measurements.
     */
    public void localizeRobotPose() {
        // Measure vision processing time
        double visionStart = Timer.getFPGATimestamp();
        
        var photonVision = m_subsystemFactory.getPhotonVision();
        
        // Get estimated robot pose from PhotonVision
        var estimatedPose = photonVision.getEstimatedRobotPose();
        
        if (estimatedPose.isPresent()) {
            var pose = estimatedPose.get();
            
            // Get the pose in the correct alliance coordinate system
            // PhotonVision returns poses in the field's native coordinate system.
            // For rotationally symmetrical fields, PhotonVision should handle the
            // coordinate transformation automatically, but we verify the pose is correct.
            Pose2d visionPose = pose.estimatedPose.toPose2d();
            
            // Add vision measurement to drivetrain pose estimator
            // Timestamp is already in seconds from PhotonVision
            m_drivetrain.addVisionMeasurement(
                visionPose,
                pose.timestampSeconds
            );
            
            // Log vision pose for debugging
            RobotLogger.recordPose2d("Vision/Pose", visionPose);
        }
        
        // Log robot pose (from drivetrain odometry)
        RobotLogger.recordPose2d("Drivetrain/Pose", m_drivetrain.getPose());
        
        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // Log mechanism positions
        // var context = m_subsystemFactory.getRobotContext();
        // Removed for 2026:
        // RobotLogger.recordDouble("Mechanisms/ElevatorPosition", m_subsystemFactory.getElevator().getPosition());
        // RobotLogger.recordDouble("Mechanisms/ArmAngle", m_subsystemFactory.getArm().getPosition());
        // RobotLogger.recordDouble("Mechanisms/SideToSidePosition", m_subsystemFactory.getSideToSide().getPosition());
        // RobotLogger.recordDouble("Mechanisms/ClimberPosition", m_subsystemFactory.getClimber().getPosition());
        
        // Log sensor states
        // var positionTracker = m_subsystemFactory.getPositionTracker();
        // RobotLogger.recordBoolean("Sensors/CoralInTray", positionTracker.getCoralInTray());
        // RobotLogger.recordBoolean("Sensors/CoralInArm", positionTracker.getCoralInArm());
        
        // Log current state
        // RobotLogger.recordString("State/CurrentState", context.stateMachine.getCurrentState().Name.toString());
        
        // Record vision processing time
        double visionTime = Timer.getFPGATimestamp() - visionStart;
        PerformanceMonitor.getInstance().recordVisionTime(visionTime);
    }

    // Getters for subsystems (for backwards compatibility and external access)
    // Removed for 2026:
    // public Elevator getElevator() {
    //     return m_subsystemFactory.getElevator();
    // }

    // public Arm getArm() {
    //     return m_subsystemFactory.getArm();
    // }

    // public SideToSide getSideToSide() {
    //     return m_subsystemFactory.getSideToSide();
    // }

    // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
    // public Climber getClimber() {
    //     return m_subsystemFactory.getClimber();
    // }

    // public Lights getLights() {
    //     return m_subsystemFactory.getLights();
    // }

    // public PositionTracker getPositionTracker() {
    //     return m_subsystemFactory.getPositionTracker();
    // }

    // public RobotContext getRobotContext() {
    //     return m_subsystemFactory.getRobotContext();
    // }

    public AutoManager getAutoManager() {
        return m_autoManager;
    }

    /**
     * Cleans up all resources when the robot is disabled.
     * Stops all motors, closes NetworkTables publishers, and closes sensors.
     */
    public void cleanup() {
        // Stop drivetrain
        if (m_drivetrain != null) {
            m_drivetrain.stop();
        }

        // Reset rate limiters to prevent stale state
        SwerveDrivetrainBindings.resetRateLimiters();

        // Cleanup all subsystems and sensors
        if (m_subsystemFactory != null) {
            m_subsystemFactory.cleanup();
        }
    }
}
