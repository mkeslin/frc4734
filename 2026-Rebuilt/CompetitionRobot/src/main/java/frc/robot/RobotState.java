package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Manages global robot state flags.
 * Replaces the mutable enum pattern with a proper class-based design for better
 * maintainability, testability, and extensibility.
 * 
 * State changes are logged for debugging and troubleshooting.
 */
public class RobotState {
    private static RobotState instance;
    
    private boolean isInitialized;
    
    /**
     * Private constructor to enforce singleton pattern.
     * Initializes state based on whether running on real robot or simulation.
     */
    private RobotState() {
        // In simulation, start initialized; on real robot, require explicit initialization
        isInitialized = !RobotBase.isReal();
        if (isInitialized) {
            DataLogManager.log("[RobotState] Initialized (simulation mode)");
        } else {
            DataLogManager.log("[RobotState] Not initialized (real robot - requires explicit initialization)");
        }
    }
    
    /**
     * Gets the singleton RobotState instance.
     * 
     * @return the RobotState instance
     */
    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
    
    /**
     * Checks if the robot is initialized.
     * 
     * @return true if initialized, false otherwise
     */
    public boolean isInitialized() {
        return isInitialized;
    }
    
    /**
     * Sets the initialized state.
     * 
     * @param initialized true to mark as initialized, false otherwise
     */
    private void setInitialized(boolean initialized) {
        if (this.isInitialized != initialized) {
            this.isInitialized = initialized;
            DataLogManager.log(String.format("[RobotState] Initialization state changed: %s", initialized ? "INITIALIZED" : "NOT INITIALIZED"));
        }
    }
    
    /**
     * Returns a command that enables initialization when executed.
     * 
     * @return Command to enable initialization
     */
    public Command enableInitializationCommand() {
        return Commands.runOnce(() -> setInitialized(true)).ignoringDisable(true);
    }
    
    /**
     * Returns a command that disables initialization when executed.
     * 
     * @return Command to disable initialization
     */
    public Command disableInitializationCommand() {
        return Commands.runOnce(() -> setInitialized(false)).ignoringDisable(true);
    }
    
    /**
     * Returns a command that toggles initialization state when executed.
     * 
     * @return Command to toggle initialization
     */
    public Command toggleInitializationCommand() {
        return Commands.runOnce(() -> setInitialized(!isInitialized)).ignoringDisable(true);
    }
    
    /**
     * Resets the RobotState instance (useful for testing).
     * This should only be called in test code.
     */
    public static void resetInstance() {
        instance = null;
    }
}
