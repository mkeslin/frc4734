package frc.robot.pitcheck.steps;

import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;

/**
 * Shooter spin test - spins shooter to modest RPM and verifies it reaches target.
 */
public abstract class ShooterSpinTestStep implements PitCheckStep {
    private double startTime;
    private boolean rpmChecked;
    private double maxRpmSeen;

    @Override
    public String getName() {
        return "Shooter Spin Test";
    }

    @Override
    public double getTimeout() {
        return PitCheckConstants.SHOOTER_SPIN_DURATION + 1.0;
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return true;
    }

    @Override
    public boolean initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        rpmChecked = false;
        maxRpmSeen = 0.0;
        return true;
    }

    @Override
    public void run(double elapsedTime) {
        if (elapsedTime < PitCheckConstants.SHOOTER_SPIN_DURATION) {
            // Command RPM
            setShooterRPM(PitCheckConstants.SHOOTER_TEST_RPM);

            // Check RPM after delay
            if (!rpmChecked && elapsedTime > PitCheckConstants.SHOOTER_RPM_CHECK_DELAY) {
                double currentRpm = getShooterRPM();
                if (currentRpm > maxRpmSeen) {
                    maxRpmSeen = currentRpm;
                }
                rpmChecked = true;
            }
        } else {
            // Stop shooter
            stopShooter();
        }
    }

    @Override
    public boolean isComplete(double elapsedTime) {
        return elapsedTime >= PitCheckConstants.SHOOTER_SPIN_DURATION + 0.2;
    }

    @Override
    public PitCheckResult evaluate(double duration) {
        // Recheck RPM one more time
        double finalRpm = getShooterRPM();
        if (finalRpm > maxRpmSeen) {
            maxRpmSeen = finalRpm;
        }

        if (maxRpmSeen < PitCheckConstants.SHOOTER_MIN_RPM) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                String.format("RPM too low (%.0f, min: %.0f)", maxRpmSeen, PitCheckConstants.SHOOTER_MIN_RPM),
                duration
            );
        }

        if (maxRpmSeen < PitCheckConstants.SHOOTER_TEST_RPM * 0.8) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.WARN,
                String.format("RPM below target (%.0f, target: %.0f)", maxRpmSeen, PitCheckConstants.SHOOTER_TEST_RPM),
                duration
            );
        }

        return new PitCheckResult(
            getName(),
            PitCheckResult.Status.PASS,
            String.format("RPM: %.0f", maxRpmSeen),
            duration
        );
    }

    @Override
    public void cleanup() {
        stopShooter();
    }

    // Abstract methods
    protected abstract void setShooterRPM(double rpm);
    protected abstract double getShooterRPM();
    protected abstract void stopShooter();
}
