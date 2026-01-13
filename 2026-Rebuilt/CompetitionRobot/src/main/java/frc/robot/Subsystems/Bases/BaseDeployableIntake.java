package frc.robot.Subsystems.Bases;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base scaffolding for a deployable intake mechanism.
 * Combines position control for deploy/stow functionality with velocity control for intake operation.
 * 
 * <p>
 * To use, implement {@code BaseDeployableIntake<DeployPosition, IntakeSpeed>}, where
 * {@code DeployPosition} is an enum with deploy/stow positions and {@code IntakeSpeed} is an enum
 * with intake speed setpoints.
 * <p>
 * Examples of deployable intake mechanisms include: rotational-deployment intakes,
 * telescoping intakes, and pivoting intakes.
 */
public interface BaseDeployableIntake<DeployPosition extends Enum<DeployPosition>, IntakeSpeed extends Enum<IntakeSpeed>> {
    // Deploy position methods (similar to BaseSingleJointedArm)
    
    /**
     * Gets the current deploy position of the mechanism.
     * 
     * @return the current deploy position, in rotations
     */
    double getDeployPosition();

    /**
     * Resets the deploy position to a specific value (this should be the position of a hard stop).
     */
    void resetDeployPosition();

    /**
     * Creates a command that moves the deploy mechanism to a setpoint position.
     * 
     * @param goalPositionSupplier a supplier of an instance of the deploy position enum
     * @return the command
     */
    Command moveToSetDeployPositionCommand(Supplier<DeployPosition> goalPositionSupplier);

    /**
     * Creates a command that moves the deploy mechanism to an arbitrary position.
     * 
     * @param goalPositionSupplier a supplier of a position to move to, in rotations
     * @return the command
     */
    Command moveToArbitraryDeployPositionCommand(Supplier<Double> goalPositionSupplier);

    /**
     * Creates a command that moves the deploy mechanism by a delta amount.
     * 
     * @param delta a supplier of a delta to move, in rotations
     * @return the command
     */
    Command moveDeployPositionDeltaCommand(Supplier<Double> delta);

    /**
     * Creates an instantaneous command that resets the deploy position.
     * 
     * @return the command
     */
    Command resetDeployPositionCommand();

    /**
     * Checks if the intake is currently deployed (not stowed).
     * 
     * @return true if deployed, false if stowed
     */
    boolean isDeployed();

    // Intake speed methods (similar to BaseIntake)
    
    /**
     * Gets the current intake speed.
     * 
     * @return the current intake speed
     */
    double getIntakeSpeed();

    /**
     * Resets the intake speed to stopped.
     */
    void resetIntakeSpeed();

    /**
     * Creates a command that sets the intake to a predefined speed.
     * 
     * @param goalSpeedSupplier a supplier of an instance of the intake speed enum
     * @return the command
     */
    Command moveToSetIntakeSpeedCommand(Supplier<IntakeSpeed> goalSpeedSupplier);

    /**
     * Creates a command that sets the intake to an arbitrary speed.
     * 
     * @param goalSpeedSupplier a supplier of a speed value
     * @return the command
     */
    Command moveToArbitraryIntakeSpeedCommand(Supplier<Double> goalSpeedSupplier);

    /**
     * Creates a command that adjusts the intake speed by a delta amount.
     * 
     * @param delta a supplier of a delta to adjust the speed
     * @return the command
     */
    Command moveIntakeSpeedDeltaCommand(Supplier<Double> delta);

    /**
     * Creates an instantaneous command that resets the intake speed to stopped.
     * 
     * @return the command
     */
    Command resetIntakeSpeedCommand();

    // Coordination methods
    
    /**
     * Creates a command that stops both motors and sets them to coast mode,
     * to allow for moving the mechanism manually.
     * 
     * @apiNote use
     *          {@code .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)}
     *          for safety
     * @return the command
     */
    Command coastMotorsCommand();
}
