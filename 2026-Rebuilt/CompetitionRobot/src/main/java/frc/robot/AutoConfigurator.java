package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.AutoManager;

/**
 * Configures autonomous routines for the robot.
 * Centralizes autonomous setup logic.
 * 
 * <p>NOTE: This class is currently a placeholder. The old 2025 auto commands have been removed.
 * When ready to enable 2026 auto routines, update this class to use the new atomic commands
 * from frc.robot.Auto.commands.AutoRoutines.
 * 
 * <p>Example usage (when ready):
 * <pre>
 * Command climberAuto = AutoRoutines.buildClimberAuto(...);
 * m_autoManager.addRoutine(new AutoRoutine("ClimberAuto", climberAuto));
 * </pre>
 */
public class AutoConfigurator {
    private final AutoManager m_autoManager;

    /**
     * Creates a new AutoConfigurator with required dependencies.
     * 
     * @param autoManager The AutoManager instance
     */
    public AutoConfigurator(AutoManager autoManager) {
        m_autoManager = autoManager;
    }

    /**
     * Configures all autonomous routines and registers them with AutoManager.
     * 
     * <p>Currently empty - update this method to register 2026 auto routines
     * using the atomic commands from frc.robot.Auto.commands.AutoRoutines.
     */
    public void configureAuto() {
        // TODO: Register 2026 auto routines using AutoRoutines builder
        // Example:
        // Command climberAuto = AutoRoutines.buildClimberAuto(...);
        // m_autoManager.addRoutine(new AutoRoutine("ClimberAuto", climberAuto));
        
        SmartDashboard.putData("Auto Mode (manager)", m_autoManager.chooser);
    }
}
