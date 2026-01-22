package frc.robot.pitcheck.steps;

import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;

/**
 * Vision heartbeat test - verifies camera is alive and optionally sees tags.
 */
public abstract class VisionHeartbeatTestStep implements PitCheckStep {
    @Override
    public String getName() {
        return "Vision Heartbeat Test";
    }

    @Override
    public double getTimeout() {
        return 2.0;
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return true;
    }

    @Override
    public boolean initialize() {
        return true;
    }

    @Override
    public void run(double elapsedTime) {
        // No motion needed
    }

    @Override
    public boolean isComplete(double elapsedTime) {
        return elapsedTime > 0.1;
    }

    @Override
    public PitCheckResult evaluate(double duration) {
        // Check camera connection
        if (!isCameraConnected()) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                "Camera not connected",
                duration
            );
        }

        // Check last update time
        double lastUpdateAge = getLastUpdateAge();
        if (lastUpdateAge > PitCheckConstants.VISION_MAX_AGE) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.WARN,
                String.format("Camera data stale (%.1fs old)", lastUpdateAge),
                duration
            );
        }

        // Optionally check tag count
        int tagCount = getTagCount();
        if (PitCheckConstants.VISION_MIN_TAG_COUNT > 0 && tagCount < PitCheckConstants.VISION_MIN_TAG_COUNT) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.WARN,
                String.format("No tags visible (count: %d)", tagCount),
                duration
            );
        }

        return new PitCheckResult(
            getName(),
            PitCheckResult.Status.PASS,
            String.format("Camera alive, tags: %d", tagCount),
            duration
        );
    }

    @Override
    public void cleanup() {
        // Nothing to clean up
    }

    // Abstract methods
    protected abstract boolean isCameraConnected();
    protected abstract double getLastUpdateAge(); // seconds since last update
    protected abstract int getTagCount();
}
