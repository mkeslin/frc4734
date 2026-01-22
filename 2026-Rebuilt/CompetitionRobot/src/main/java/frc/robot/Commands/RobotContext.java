package frc.robot.Commands;

import java.util.Objects;

import frc.robot.PositionTracker;
import frc.robot.State.StateMachine;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Context object containing all common dependencies for robot commands.
 * This reduces parameter count and improves code readability.
 */
public class RobotContext {
    public final StateMachine stateMachine;
    public final PositionTracker positionTracker;
    public final CommandSwerveDrivetrain drivetrain;
    public final DeployableIntake deployableIntake;
    public final Floor floor;
    public final Feeder feeder;
    public final Shooter shooter;
    public final Lights lights;
    public final PhotonVision photonVision;

    /**
     * Creates a new RobotContext with all required dependencies.
     * 
     * @param stateMachine The state machine for managing robot states
     * @param positionTracker The position tracker for mechanism positions
     * @param drivetrain The swerve drivetrain
     * @param deployableIntake The deployable intake subsystem
     * @param floor The floor conveyor subsystem
     * @param feeder The feeder subsystem
     * @param shooter The shooter subsystem
     * @param lights The lights subsystem
     * @param photonVision The PhotonVision camera
     * @throws NullPointerException if any parameter is null
     */
    public RobotContext(
            StateMachine stateMachine,
            PositionTracker positionTracker,
            CommandSwerveDrivetrain drivetrain,
            DeployableIntake deployableIntake,
            Floor floor,
            Feeder feeder,
            Shooter shooter,
            Lights lights,
            PhotonVision photonVision) {
        this.stateMachine = Objects.requireNonNull(stateMachine, "StateMachine cannot be null");
        this.positionTracker = Objects.requireNonNull(positionTracker, "PositionTracker cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        this.deployableIntake = Objects.requireNonNull(deployableIntake, "DeployableIntake cannot be null");
        this.floor = Objects.requireNonNull(floor, "Floor cannot be null");
        this.feeder = Objects.requireNonNull(feeder, "Feeder cannot be null");
        this.shooter = Objects.requireNonNull(shooter, "Shooter cannot be null");
        this.lights = Objects.requireNonNull(lights, "Lights cannot be null");
        this.photonVision = Objects.requireNonNull(photonVision, "PhotonVision cannot be null");
    }
}
