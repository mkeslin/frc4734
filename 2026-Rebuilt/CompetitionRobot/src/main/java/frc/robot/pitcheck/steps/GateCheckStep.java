package frc.robot.pitcheck.steps;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;

/**
 * Gate check step - verifies safety conditions before running tests.
 * No motion, just checks DriverStation state and required toggles.
 */
public class GateCheckStep implements PitCheckStep {
    @Override
    public String getName() {
        return "Gate Check";
    }

    @Override
    public double getTimeout() {
        return PitCheckConstants.GATE_CHECK_TIMEOUT;
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return true; // Always run gate check
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
        return elapsedTime > 0.1; // Quick check
    }

    @Override
    public PitCheckResult evaluate(double duration) {
        // Check FMS
        if (DriverStation.isFMSAttached()) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                "FMS is attached - pit check disabled",
                duration
            );
        }

        // Check enabled state (should be Disabled or Test)
        if (DriverStation.isEnabled()) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                "Robot is enabled - must be Disabled or Test mode",
                duration
            );
        }

        // All gates passed
        return new PitCheckResult(
            getName(),
            PitCheckResult.Status.PASS,
            "All safety gates passed",
            duration
        );
    }

    @Override
    public void cleanup() {
        // Nothing to clean up
    }
}
