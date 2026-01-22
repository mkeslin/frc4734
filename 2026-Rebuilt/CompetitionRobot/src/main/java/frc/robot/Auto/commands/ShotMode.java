package frc.robot.Auto.commands;

/**
 * Enumeration of shooter shot modes.
 * Represents intent only - no embedded RPM values or tuning constants.
 * 
 * <p>Shot mode selection determines which shot model or fallback behavior to use.
 * Actual RPM values, hood angles, and other tuning parameters are handled by
 * a separate shot model/calculator that considers distance, vision quality,
 * battery voltage, and tuning iteration.
 * 
 * <p>This design keeps tuning flexible and prevents the enum from becoming
 * a dumping ground for hardcoded values.
 */
public enum ShotMode {
    /**
     * Autonomous shot mode.
     * Uses aggressive shot model optimized for autonomous precision.
     */
    AUTO_SHOT,

    /**
     * Safe shot mode.
     * Uses conservative shot model with higher margin for error.
     */
    SAFE_SHOT
}
