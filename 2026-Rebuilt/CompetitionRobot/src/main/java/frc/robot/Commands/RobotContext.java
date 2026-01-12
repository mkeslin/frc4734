package frc.robot.Commands;

import frc.robot.PositionTracker;
import frc.robot.State.StateMachine;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Context object containing all common dependencies for robot commands.
 * This reduces parameter count and improves code readability.
 */
public class RobotContext {
    public final StateMachine stateMachine;
    public final PositionTracker positionTracker;
    public final CommandSwerveDrivetrain drivetrain;
    public final Elevator elevator;
    public final Arm arm;
    public final SideToSide sideToSide;
    public final Lights lights;
    public final Limelight reefLimelight;

    /**
     * Creates a new RobotContext with all required dependencies.
     * 
     * @param stateMachine The state machine for managing robot states
     * @param positionTracker The position tracker for mechanism positions
     * @param drivetrain The swerve drivetrain
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param sideToSide The side-to-side subsystem
     * @param lights The lights subsystem
     * @param reefLimelight The reef limelight camera
     */
    public RobotContext(
            StateMachine stateMachine,
            PositionTracker positionTracker,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight) {
        this.stateMachine = stateMachine;
        this.positionTracker = positionTracker;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.arm = arm;
        this.sideToSide = sideToSide;
        this.lights = lights;
        this.reefLimelight = reefLimelight;
    }
}
