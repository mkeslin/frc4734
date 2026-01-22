package frc.robot.pitcheck.steps;

import java.util.List;
import java.util.function.Supplier;

import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;
import frc.robot.pitcheck.PitCheckTelemetry;

/**
 * Drive module pulse test - pulses drive motors forward at low power.
 * 
 * This is an abstract base class. Subclasses must provide:
 * - A way to command chassis speeds
 * - Encoder position/velocity suppliers
 * - Current suppliers (optional)
 */
public abstract class DrivePulseTestStep implements PitCheckStep {
    private double startTime;
    private double startPosition;
    private List<PitCheckTelemetry.Sample> samples;
    private boolean samplesCollected;

    @Override
    public String getName() {
        return "Drive Pulse Test";
    }

    @Override
    public double getTimeout() {
        return PitCheckConstants.DRIVE_PULSE_DURATION + 0.5;
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return robotOnBlocks; // Requires blocks
    }

    @Override
    public boolean initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        startPosition = getPositionSupplier().get();
        samples = null;
        samplesCollected = false;
        return true;
    }

    @Override
    public void run(double elapsedTime) {
        if (elapsedTime < PitCheckConstants.DRIVE_PULSE_DURATION) {
            // Pulse forward
            commandChassisSpeeds(
                PitCheckConstants.DRIVE_PULSE_POWER, // forward
                0.0, // sideways
                0.0  // rotation
            );

            // Collect samples
            if (!samplesCollected) {
                samples = PitCheckTelemetry.collectSamples(
                    PitCheckConstants.DRIVE_PULSE_DURATION,
                    PitCheckConstants.TELEMETRY_SAMPLE_RATE,
                    getPositionSupplier(),
                    getVelocitySupplier(),
                    getCurrentSupplier(),
                    getVoltageSupplier()
                );
                samplesCollected = true;
            }
        } else {
            // Stop
            commandChassisSpeeds(0.0, 0.0, 0.0);
        }
    }

    @Override
    public boolean isComplete(double elapsedTime) {
        return elapsedTime >= PitCheckConstants.DRIVE_PULSE_DURATION + 0.1;
    }

    @Override
    public PitCheckResult evaluate(double duration) {
        if (samples == null || samples.size() < PitCheckConstants.MIN_SAMPLES_FOR_EVALUATION) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                "Insufficient telemetry samples",
                duration
            );
        }

        double encoderDelta = PitCheckTelemetry.calculateEncoderDelta(samples);
        double avgCurrent = PitCheckTelemetry.calculateAverageCurrent(samples);
        double peakCurrent = PitCheckTelemetry.calculatePeakCurrent(samples);

        // Check encoder movement
        if (encoderDelta < PitCheckConstants.DRIVE_MIN_ENCODER_DELTA) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                String.format("Encoder did not move (delta: %.4f)", encoderDelta),
                duration
            );
        }

        // Check current if available
        if (getCurrentSupplier() != null) {
            if (avgCurrent < PitCheckConstants.DRIVE_MIN_CURRENT) {
                return new PitCheckResult(
                    getName(),
                    PitCheckResult.Status.FAIL,
                    String.format("Current too low (%.2fA) - motor may be unplugged", avgCurrent),
                    duration
                );
            }
            if (peakCurrent > PitCheckConstants.DRIVE_MAX_CURRENT) {
                return new PitCheckResult(
                    getName(),
                    PitCheckResult.Status.WARN,
                    String.format("Current high (peak: %.2fA) - may be jammed", peakCurrent),
                    duration
                );
            }
        }

        return new PitCheckResult(
            getName(),
            PitCheckResult.Status.PASS,
            String.format("Encoder delta: %.4f, Current: %.2fA", encoderDelta, avgCurrent),
            duration
        );
    }

    @Override
    public void cleanup() {
        commandChassisSpeeds(0.0, 0.0, 0.0);
    }

    // Abstract methods to be implemented
    protected abstract void commandChassisSpeeds(double vx, double vy, double omega);
    protected abstract Supplier<Double> getPositionSupplier();
    protected abstract Supplier<Double> getVelocitySupplier();
    protected abstract Supplier<Double> getCurrentSupplier();
    protected abstract Supplier<Double> getVoltageSupplier();
}
