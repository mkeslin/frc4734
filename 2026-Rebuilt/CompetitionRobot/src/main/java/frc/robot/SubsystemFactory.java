package frc.robot;

import static frc.robot.Constants.DigitalInputIds.CORAL_ARM_SENSOR;
import static frc.robot.Constants.DigitalInputIds.CORAL_TRAY_SENSOR;
import static frc.robot.Constants.VisionConstants.APRILTAG_PIPELINE;
import static frc.robot.Constants.VisionConstants.CAMERA_NAME;

import java.util.Objects;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Commands.RobotContext;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterHood;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.State.StateMachine;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Factory class for creating and initializing all robot subsystems and related components.
 * Centralizes subsystem creation logic and ensures proper initialization order.
 */
public class SubsystemFactory {
    // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
    // private final Climber m_climber;
    private final DeployableIntake m_deployableIntake;
    private final Floor m_floor;
    private final Feeder m_feeder;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    // private final Lights m_lights;
    private final PhotonVision m_photonVision;
    // private final DigitalInput m_coralTraySensor;
    // private final DigitalInput m_coralArmSensor;
    // private final PositionTracker m_positionTracker;
    // private final StateMachine m_stateMachine;
    // private final RobotContext m_robotContext;
    private final CommandSwerveDrivetrain m_drivetrain;

    /**
     * Creates a new SubsystemFactory and initializes all subsystems, sensors, and related components.
     * 
     * @param drivetrain The swerve drivetrain instance
     * @throws NullPointerException if drivetrain is null
     */
    public SubsystemFactory(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");

        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // Create subsystems (PositionTracker will be set after creation)
        // m_climber = new Climber();
        m_deployableIntake = new DeployableIntake();
        m_floor = new Floor();
        m_feeder = new Feeder();
        m_shooter = new Shooter();
        m_shooterHood = new ShooterHood();
        // m_lights = new Lights();

        // Create sensors
        // m_coralTraySensor = new DigitalInput(CORAL_TRAY_SENSOR);
        // m_coralArmSensor = new DigitalInput(CORAL_ARM_SENSOR);

        // Create PhotonVision camera
        m_photonVision = new PhotonVision(CAMERA_NAME, APRILTAG_PIPELINE);

        // Create PositionTracker with all actual suppliers
        // Method references will work correctly since subsystems are now created
        // m_positionTracker = new PositionTracker(
        //         m_climber::getPosition,
        //         m_coralTraySensor::get,
        //         m_coralArmSensor::get,
        //         m_deployableIntake::getDeployPosition,
        //         m_deployableIntake::getIntakeSpeed,
        //         m_floor::getSpeed,
        //         m_feeder::getSpeed,
        //         m_shooter::getSpeed
        // );

        // Set PositionTracker on all subsystems
        // This ensures all subsystems share the same PositionTracker instance with real suppliers,
        // allowing them to query each other's state correctly
        // m_climber.setPositionTracker(m_positionTracker);
        // m_deployableIntake.setPositionTracker(m_positionTracker);
        // m_floor.setPositionTracker(m_positionTracker);
        // m_feeder.setPositionTracker(m_positionTracker);
        // m_shooter.setPositionTracker(m_positionTracker);

        // Create state machine (loads states automatically in constructor)
        // m_stateMachine = new StateMachine();

        // Create robot context with all dependencies
        // m_robotContext = new RobotContext(
        //         m_stateMachine,
        //         m_positionTracker,
        //         m_drivetrain,
        //         m_deployableIntake,
        //         m_floor,
        //         m_feeder,
        //         m_shooter,
        //         m_lights,
        //         m_photonVision);
    }

    // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
    // public Climber getClimber() {
    //     return m_climber;
    // }

    public DeployableIntake getDeployableIntake() {
         return m_deployableIntake;
    }

    public Floor getFloor() {
        return m_floor;
    }

    public Feeder getFeeder() {
        return m_feeder;
    }

    public Shooter getShooter() {
        return m_shooter;
    }

    public ShooterHood getShooterHood() {
        return m_shooterHood;
    }

    // public Lights getLights() {
    //     return m_lights;
    // }

    public PhotonVision getPhotonVision() {
        return m_photonVision;
    }

    // public PositionTracker getPositionTracker() {
    //     return m_positionTracker;
    // }

    // public StateMachine getStateMachine() {
    //     return m_stateMachine;
    // }

    // public RobotContext getRobotContext() {
    //     return m_robotContext;
    // }

    /**
     * Resets all subsystem positions to their zero positions.
     */
    public void resetZeros() {
        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // m_climber.resetPosition();
        // m_deployableIntake.resetDeployPosition();
        m_floor.resetSpeed();
        m_feeder.resetSpeed();
        m_shooter.resetSpeed();
        m_shooterHood.resetPosition();
    }

    /**
     * Cleans up all resources when the robot is disabled.
     * Stops all motors, closes NetworkTables publishers, closes sensors,
     * and cleans up other subsystem resources.
     */
    public void cleanup() {
        // TEMPORARILY COMMENTED OUT FOR DRIVETRAIN-ONLY TESTING
        // Cleanup subsystems
        // if (m_climber != null) {
        //     m_climber.cleanup();
        // }
        // if (m_deployableIntake != null) {
        //     m_deployableIntake.cleanup();
        // }
        if (m_floor != null) {
            m_floor.cleanup();
        }
        if (m_feeder != null) {
            m_feeder.cleanup();
        }
        if (m_shooter != null) {
            m_shooter.cleanup();
        }
        if (m_shooterHood != null) {
            m_shooterHood.cleanup();
        }
        // if (m_lights != null) {
        //     m_lights.cleanup();
        // }
        if (m_photonVision != null) {
            m_photonVision.cleanup();
        }

        // Close sensors
        // if (m_coralTraySensor != null) {
        //     m_coralTraySensor.close();
        // }
        // if (m_coralArmSensor != null) {
        //     m_coralArmSensor.close();
        // }
    }
}
