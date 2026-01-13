package frc.robot.Logging;

/**
 * Configuration class for controlling logging throughout the robot codebase.
 * 
 * <p>This class provides compile-time flags to enable/disable different categories
 * of logging. This allows fine-grained control over what gets logged, which is
 * useful for:
 * <ul>
 *   <li>Reducing overhead during competition matches</li>
 *   <li>Enabling verbose logging during testing and practice</li>
 *   <li>Optimizing performance by disabling unnecessary logging</li>
 * </ul>
 * 
 * <p>To change logging behavior, modify the boolean flags below and rebuild the code.
 * 
 * <p><b>Recommended Settings:</b>
 * <ul>
 *   <li><b>Testing/Practice:</b> Enable all flags (default)</li>
 *   <li><b>Competition:</b> Disable PERFORMANCE and MESSAGE logging, keep TELEMETRY and ERROR</li>
 *   <li><b>Maximum Performance:</b> Disable all except ERROR logging</li>
 * </ul>
 */
public final class LoggingConfig {
    private LoggingConfig() {
        // Utility class - prevent instantiation
    }

    /**
     * Controls performance metrics logging (loop timing, scheduler overhead, vision time, etc.).
     * 
     * <p><b>Testing/Practice:</b> Enable to monitor robot performance and identify bottlenecks.
     * Useful for debugging loop overruns and optimizing code execution time.
     * 
     * <p><b>Competition:</b> Disable to reduce loop overhead (~0.1-0.5ms per loop).
     * Performance metrics are still tracked internally but not logged to AdvantageKit.
     * 
     * <p><b>Default:</b> true (enabled for development)
     */
    public static final boolean ENABLE_PERFORMANCE_LOGGING = true;  // Testing: ON | Competition: OFF

    /**
     * Controls telemetry data logging (poses, sensor values, mechanism positions, etc.).
     * 
     * <p><b>Testing/Practice:</b> Enable for debugging and tuning. Essential for AdvantageScope
     * analysis to understand robot behavior and tune PID controllers.
     * 
     * <p><b>Competition:</b> Enable to maintain match replay capabilities and post-match analysis.
     * Telemetry data is critical for understanding what happened during matches and debugging issues.
     * 
     * <p><b>Default:</b> true (enabled for both testing and competition)
     */
    public static final boolean ENABLE_TELEMETRY_LOGGING = true;  // Testing: ON | Competition: ON

    /**
     * Controls informational message logging (log, logWarning methods).
     * 
     * <p><b>Testing/Practice:</b> Enable for detailed debugging and state tracking.
     * Useful for understanding robot state transitions and debugging command execution.
     * 
     * <p><b>Competition:</b> Disable to reduce NetworkTables bandwidth and string formatting overhead.
     * Informational messages are less critical than errors and can be disabled to save resources.
     * 
     * <p><b>Default:</b> true (enabled for development)
     */
    public static final boolean ENABLE_MESSAGE_LOGGING = true;  // Testing: ON | Competition: OFF

    /**
     * Controls error message logging (logError method).
     * 
     * <p><b>Testing/Practice:</b> Enable for debugging issues and understanding failures.
     * Essential for identifying problems during development and testing.
     * 
     * <p><b>Competition:</b> Enable to capture critical errors during matches.
     * Error logging is important for post-match analysis and understanding match failures.
     * 
     * <p><b>Default:</b> true (enabled for both testing and competition)
     */
    public static final boolean ENABLE_ERROR_LOGGING = true;  // Testing: ON | Competition: ON

    /**
     * If true, errors are always logged even if ENABLE_ERROR_LOGGING is false.
     * 
     * <p>This provides a safety mechanism to ensure critical errors are never missed,
     * even if error logging is accidentally disabled.
     * 
     * <p><b>Testing/Practice:</b> Set to true (default) to ensure all errors are captured.
     * 
     * <p><b>Competition:</b> Set to true to ensure critical errors are never missed during matches.
     * This acts as a failsafe in case ENABLE_ERROR_LOGGING is accidentally set to false.
     * 
     * <p><b>Default:</b> true (errors always logged for safety)
     */
    public static final boolean ALWAYS_LOG_ERRORS = true;  // Testing: ON | Competition: ON
}
