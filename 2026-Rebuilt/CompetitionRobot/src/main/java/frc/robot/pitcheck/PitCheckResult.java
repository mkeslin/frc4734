package frc.robot.pitcheck;

import java.util.Objects;

/**
 * Represents the result of a single pit check step.
 * 
 * Results can be:
 * - PASS: Step completed successfully, all checks passed
 * - WARN: Step completed but some values were marginal (e.g., current slightly low)
 * - FAIL: Step failed (motor unplugged, encoder not moving, timeout, etc.)
 */
public class PitCheckResult {
    public enum Status {
        PASS,
        WARN,
        FAIL
    }

    private final String stepName;
    private final Status status;
    private final String message;
    private final double duration;
    private final long timestamp;

    /**
     * Creates a new pit check result.
     * 
     * @param stepName Name of the step that produced this result
     * @param status Result status (PASS/WARN/FAIL)
     * @param message Human-readable message explaining the result
     * @param duration How long the step took (seconds)
     */
    public PitCheckResult(String stepName, Status status, String message, double duration) {
        this.stepName = Objects.requireNonNull(stepName, "stepName cannot be null");
        this.status = Objects.requireNonNull(status, "status cannot be null");
        this.message = Objects.requireNonNull(message, "message cannot be null");
        this.duration = duration;
        this.timestamp = System.currentTimeMillis();
    }

    public String getStepName() {
        return stepName;
    }

    public Status getStatus() {
        return status;
    }

    public String getMessage() {
        return message;
    }

    public double getDuration() {
        return duration;
    }

    public long getTimestamp() {
        return timestamp;
    }

    /**
     * Returns true if this result indicates a problem (WARN or FAIL).
     */
    public boolean hasIssue() {
        return status == Status.WARN || status == Status.FAIL;
    }

    /**
     * Returns true if this result is a critical failure.
     */
    public boolean isFailure() {
        return status == Status.FAIL;
    }

    @Override
    public String toString() {
        return String.format("%s [%s]: %s (%.2fs)", stepName, status, message, duration);
    }
}
