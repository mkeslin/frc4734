package frc.robot.autotest;

import java.util.Map;

/**
 * Record containing the result of a test command run.
 * 
 * @param name The name of the test command
 * @param reason Why the command ended
 * @param durationSec How long the command ran (seconds)
 * @param metricsStart Telemetry snapshot at command start
 * @param metricsEnd Telemetry snapshot at command end
 * @param timestamp When the test ran (ISO format string, optional)
 */
public record CommandRunResult(
        String name,
        CommandEndReason reason,
        double durationSec,
        Map<String, Double> metricsStart,
        Map<String, Double> metricsEnd,
        String timestamp) {
    
    /**
     * Creates a CommandRunResult with current timestamp.
     * 
     * @param name The name of the test command
     * @param reason Why the command ended
     * @param durationSec How long the command ran (seconds)
     * @param metricsStart Telemetry snapshot at command start
     * @param metricsEnd Telemetry snapshot at command end
     * @return A CommandRunResult with current timestamp
     */
    public static CommandRunResult create(
            String name,
            CommandEndReason reason,
            double durationSec,
            Map<String, Double> metricsStart,
            Map<String, Double> metricsEnd) {
        String timestamp = java.time.Instant.now().toString();
        return new CommandRunResult(name, reason, durationSec, metricsStart, metricsEnd, timestamp);
    }
}
