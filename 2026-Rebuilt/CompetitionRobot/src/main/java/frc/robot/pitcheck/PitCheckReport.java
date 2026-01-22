package frc.robot.pitcheck;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Aggregates all pit check step results and provides overall summary.
 */
public class PitCheckReport {
    public enum OverallStatus {
        NOT_STARTED,
        RUNNING,
        PASSED,
        WARNINGS,
        FAILED,
        ABORTED
    }

    private final List<PitCheckResult> results;
    private final long startTime;
    private long endTime;
    private OverallStatus overallStatus;
    private String abortReason;

    public PitCheckReport() {
        this.results = new ArrayList<>();
        this.startTime = System.currentTimeMillis();
        this.endTime = 0;
        this.overallStatus = OverallStatus.NOT_STARTED;
        this.abortReason = null;
    }

    /**
     * Adds a result to the report.
     */
    public void addResult(PitCheckResult result) {
        Objects.requireNonNull(result, "result cannot be null");
        results.add(result);
    }

    /**
     * Marks the report as complete and calculates overall status.
     */
    public void complete() {
        this.endTime = System.currentTimeMillis();
        this.overallStatus = calculateOverallStatus();
    }

    /**
     * Marks the report as aborted.
     */
    public void abort(String reason) {
        this.endTime = System.currentTimeMillis();
        this.overallStatus = OverallStatus.ABORTED;
        this.abortReason = reason;
    }

    /**
     * Marks the report as running.
     */
    public void setRunning() {
        this.overallStatus = OverallStatus.RUNNING;
    }

    private OverallStatus calculateOverallStatus() {
        if (results.isEmpty()) {
            return OverallStatus.NOT_STARTED;
        }

        boolean hasFail = false;
        boolean hasWarn = false;

        for (PitCheckResult result : results) {
            if (result.isFailure()) {
                hasFail = true;
            } else if (result.getStatus() == PitCheckResult.Status.WARN) {
                hasWarn = true;
            }
        }

        if (hasFail) {
            return OverallStatus.FAILED;
        } else if (hasWarn) {
            return OverallStatus.WARNINGS;
        } else {
            return OverallStatus.PASSED;
        }
    }

    public List<PitCheckResult> getResults() {
        return Collections.unmodifiableList(results);
    }

    public long getStartTime() {
        return startTime;
    }

    public long getEndTime() {
        return endTime;
    }

    public double getTotalDuration() {
        if (endTime == 0) {
            return (System.currentTimeMillis() - startTime) / 1000.0;
        }
        return (endTime - startTime) / 1000.0;
    }

    public OverallStatus getOverallStatus() {
        return overallStatus;
    }

    public String getAbortReason() {
        return abortReason;
    }

    /**
     * Gets the result for a specific step by name.
     */
    public PitCheckResult getResultForStep(String stepName) {
        return results.stream()
                .filter(r -> r.getStepName().equals(stepName))
                .findFirst()
                .orElse(null);
    }

    /**
     * Returns true if all steps passed.
     */
    public boolean isAllPassed() {
        return overallStatus == OverallStatus.PASSED;
    }

    /**
     * Returns true if there are any failures.
     */
    public boolean hasFailures() {
        return overallStatus == OverallStatus.FAILED;
    }
}
