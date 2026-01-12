package frc.robot;

import static frc.robot.Constants.DigitalInputIds.CORAL_ARM_SENSOR;
import static frc.robot.Constants.DigitalInputIds.CORAL_TRAY_SENSOR;
import static frc.robot.Constants.VisionConstants.APRILTAG_PIPELINE;
import static frc.robot.Constants.VisionConstants.CAMERA_NAME;

import java.util.Objects;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Commands.RobotContext;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.State.StateMachine;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Factory class for creating and initializing all robot subsystems and related components.
 * Centralizes subsystem creation logic and ensures proper initialization order.
 */
public class SubsystemFactory {
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final SideToSide m_sideToSide;
    private final Climber m_climber;
    private final Lights m_lights;
    private final PhotonVision m_reefPhotonVision;
    private final DigitalInput m_coralTraySensor;
    private final DigitalInput m_coralArmSensor;
    private final PositionTracker m_positionTracker;
    private final StateMachine m_stateMachine;
    private final RobotContext m_robotContext;
    private final CommandSwerveDrivetrain m_drivetrain;

    /**
     * Creates a new SubsystemFactory and initializes all subsystems, sensors, and related components.
     * 
     * @param drivetrain The swerve drivetrain instance
     * @throws NullPointerException if drivetrain is null
     */
    public SubsystemFactory(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");

        // Create subsystems (PositionTracker will be set after creation)
        m_elevator = new Elevator();
        m_arm = new Arm(m_elevator::getCarriageComponentPose);
        m_sideToSide = new SideToSide();
        m_climber = new Climber();
        m_lights = new Lights();

        // Create sensors
        m_coralTraySensor = new DigitalInput(CORAL_TRAY_SENSOR);
        m_coralArmSensor = new DigitalInput(CORAL_ARM_SENSOR);

        // Create PhotonVision camera
        m_reefPhotonVision = new PhotonVision(CAMERA_NAME, APRILTAG_PIPELINE);

        // Create PositionTracker with all actual suppliers
        // Method references will work correctly since subsystems are now created
        m_positionTracker = new PositionTracker(
                m_elevator::getPosition,
                m_arm::getPosition,
                m_sideToSide::getPosition,
                m_climber::getPosition,
                m_coralTraySensor::get,
                m_coralArmSensor::get,
                () -> 0.0  // algaeIntake not used
        );

        // Set PositionTracker on all subsystems
        // This ensures all subsystems share the same PositionTracker instance with real suppliers,
        // allowing them to query each other's state correctly
        m_elevator.setPositionTracker(m_positionTracker);
        m_arm.setPositionTracker(m_positionTracker);
        m_sideToSide.setPositionTracker(m_positionTracker);
        m_climber.setPositionTracker(m_positionTracker);

        // Create state machine (loads states automatically in constructor)
        m_stateMachine = new StateMachine();

        // Create robot context with all dependencies
        m_robotContext = new RobotContext(
                m_stateMachine,
                m_positionTracker,
                m_drivetrain,
                m_elevator,
                m_arm,
                m_sideToSide,
                m_lights,
                m_reefPhotonVision);
    }

    public Elevator getElevator() {
        return m_elevator;
    }

    public Arm getArm() {
        return m_arm;
    }

    public SideToSide getSideToSide() {
        return m_sideToSide;
    }

    public Climber getClimber() {
        return m_climber;
    }

    public Lights getLights() {
        return m_lights;
    }

    public PhotonVision getReefPhotonVision() {
        return m_reefPhotonVision;
    }

    public PositionTracker getPositionTracker() {
        return m_positionTracker;
    }

    public StateMachine getStateMachine() {
        return m_stateMachine;
    }

    public RobotContext getRobotContext() {
        return m_robotContext;
    }

    /**
     * Resets all subsystem positions to their zero positions.
     */
    public void resetZeros() {
        m_sideToSide.resetPosition();
        m_arm.resetPosition();
        m_elevator.resetPosition();
        m_climber.resetPosition();
    }
}
