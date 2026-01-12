package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.RobotContext;
import frc.robot.Controllers.ControllerIds;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
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

    // FACTORY AND CONFIGURATORS
    private final SubsystemFactory m_subsystemFactory;
    private final BindingConfigurator m_bindingConfigurator;
    private final AutoConfigurator m_autoConfigurator;

    // COMMANDS
    public final CenterToReefCommand m_centerToReefCommand;

    public RobotContainer() {
        // Create subsystem factory (creates all subsystems, PositionTracker, StateMachine, RobotContext)
        m_subsystemFactory = new SubsystemFactory(m_drivetrain);

        // Create center to reef command
        m_centerToReefCommand = new CenterToReefCommand(
                m_subsystemFactory.getReefPhotonVision(),
                m_drivetrain,
                m_driveController,
                3);

        // Register named commands
        NamedCommands.registerCommand("centerToReefCommand", m_centerToReefCommand);

        // Configure bindings for swerve drivetrain
        SwerveDrivetrainBindings.configureBindings(m_driveController, m_drivetrain);

        // Create binding configurator and configure all controller bindings
        m_bindingConfigurator = new BindingConfigurator(
                m_driveController,
                m_mechanismController,
                m_arcadeController,
                m_subsystemFactory.getRobotContext(),
                m_subsystemFactory.getClimber(),
                m_centerToReefCommand);
        m_bindingConfigurator.configureAllBindings();

        // Create auto configurator and configure autonomous routines
        m_autoConfigurator = new AutoConfigurator(
                m_subsystemFactory.getRobotContext(),
                m_centerToReefCommand);
        m_autoConfigurator.configureAuto();

        // Seed field-centric pose
        m_drivetrain.seedFieldCentric();

        // Initialize subsystems
        GlobalStates.INITIALIZED.enableCommand();

        // Reset positions
        m_subsystemFactory.resetZeros();
    }

    /**
     * Localizes the robot pose using PhotonVision AprilTag detection.
     * Updates the drivetrain's pose estimator with vision measurements.
     */
    public void localizeRobotPose() {
        var photonVision = m_subsystemFactory.getReefPhotonVision();
        
        // Get estimated robot pose from PhotonVision
        var estimatedPose = photonVision.getEstimatedRobotPose();
        
        if (estimatedPose.isPresent()) {
            var pose = estimatedPose.get();
            
            // Get the pose in the correct alliance coordinate system
            Pose2d visionPose = pose.estimatedPose.toPose2d();
            
            // Handle alliance flipping if needed
            // PhotonVision should already handle this, but verify
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                // If field layout needs to be flipped, PhotonVision should handle it
                // But we can apply additional transformations if needed
            }
            
            // Add vision measurement to drivetrain pose estimator
            // Timestamp is already in seconds from PhotonVision
            m_drivetrain.addVisionMeasurement(
                visionPose,
                pose.timestampSeconds
            );
        }
    }

    // Getters for subsystems (for backwards compatibility and external access)
    public Elevator getElevator() {
        return m_subsystemFactory.getElevator();
    }

    public Arm getArm() {
        return m_subsystemFactory.getArm();
    }

    public SideToSide getSideToSide() {
        return m_subsystemFactory.getSideToSide();
    }

    public Climber getClimber() {
        return m_subsystemFactory.getClimber();
    }

    public Lights getLights() {
        return m_subsystemFactory.getLights();
    }

    public PositionTracker getPositionTracker() {
        return m_subsystemFactory.getPositionTracker();
    }

    public RobotContext getRobotContext() {
        return m_subsystemFactory.getRobotContext();
    }
}
