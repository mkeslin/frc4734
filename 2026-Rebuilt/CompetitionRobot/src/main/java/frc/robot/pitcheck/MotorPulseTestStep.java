package frc.robot.pitcheck;

import java.util.List;
import java.util.function.Supplier;

/**
 * Generic motor pulse test step.
 * 
 * Pulses a motor at low power for a short duration and verifies:
 * - Encoder position changes (if encoder available)
 * - Current draw is within expected range
 * 
 * This can be used for any motor subsystem that provides:
 * - A way to set motor power
 * - Encoder position/velocity suppliers
 * - Current supplier (optional)
 */
public abstract class MotorPulseTestStep implements PitCheckStep {
    protected final String name;
    protected final double pulseDuration;
    protected final double pulsePower;
    protected final double minEncoderDelta;
    protected final double minCurrent;
    protected final double maxCurrent;
    
    protected double startTime;
    protected double startPosition;
    protected List<PitCheckTelemetry.Sample> samples;
    protected boolean hasEncoder;
    protected boolean hasCurrentSensor;

    /**
     * Creates a new motor pulse test step.
     * 
     * @param name Step name
     * @param pulseDuration How long to pulse (seconds)
     * @param pulsePower Power level [-1.0, 1.0]
     * @param minEncoderDelta Minimum encoder change to pass
     * @param minCurrent Minimum current to detect motor (amps)
     * @param maxCurrent Maximum current before considering jammed (amps)
     */
    protected MotorPulseTestStep(
            String name,
            double pulseDuration,
            double pulsePower,
            double minEncoderDelta,
            double minCurrent,
            double maxCurrent) {
        this.name = name;
        this.pulseDuration = pulseDuration;
        this.pulsePower = pulsePower;
        this.minEncoderDelta = minEncoderDelta;
        this.minCurrent = minCurrent;
        this.maxCurrent = maxCurrent;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public double getTimeout() {
        return pulseDuration + 0.5; // Add buffer
    }

    @Override
    public boolean initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        startPosition = getPositionSupplier().get();
        hasEncoder = !Double.isNaN(startPosition);
        hasCurrentSensor = getCurrentSupplier() != null;
        samples = null;
        return true;
    }

    @Override
    public void run(double elapsedTime) {
        if (elapsedTime < pulseDuration) {
            // Pulse motor
            setMotorPower(pulsePower);
            
            // Collect samples
            if (samples == null) {
                samples = PitCheckTelemetry.collectSamples(
                    pulseDuration,
                    PitCheckConstants.TELEMETRY_SAMPLE_RATE,
                    getPositionSupplier(),
                    getVelocitySupplier(),
                    getCurrentSupplier(),
                    getVoltageSupplier()
                );
            }
        } else {
            // Stop motor
            setMotorPower(0.0);
        }
    }

    @Override
    public boolean isComplete(double elapsedTime) {
        return elapsedTime >= pulseDuration + 0.1; // Small delay after stopping
    }

    @Override
    public PitCheckResult evaluate(double duration) {
        if (samples == null || samples.size() < PitCheckConstants.MIN_SAMPLES_FOR_EVALUATION) {
            return new PitCheckResult(
                name,
                PitCheckResult.Status.FAIL,
                "Insufficient telemetry samples collected",
                duration
            );
        }

        double encoderDelta = PitCheckTelemetry.calculateEncoderDelta(samples);
        double avgCurrent = PitCheckTelemetry.calculateAverageCurrent(samples);
        double peakCurrent = PitCheckTelemetry.calculatePeakCurrent(samples);

        // Check encoder movement
        if (hasEncoder && encoderDelta < minEncoderDelta) {
            return new PitCheckResult(
                name,
                PitCheckResult.Status.FAIL,
                String.format("Encoder did not move (delta: %.4f, min: %.4f)", encoderDelta, minEncoderDelta),
                duration
            );
        }

        // Check current (if available)
        if (hasCurrentSensor) {
            if (avgCurrent < minCurrent) {
                return new PitCheckResult(
                    name,
                    PitCheckResult.Status.FAIL,
                    String.format("Current too low (%.2fA, min: %.2fA) - motor may be unplugged", avgCurrent, minCurrent),
                    duration
                );
            }
            if (peakCurrent > maxCurrent) {
                return new PitCheckResult(
                    name,
                    PitCheckResult.Status.WARN,
                    String.format("Current high (peak: %.2fA, max: %.2fA) - may be jammed", peakCurrent, maxCurrent),
                    duration
                );
            }
            if (avgCurrent < minCurrent * 1.2) {
                return new PitCheckResult(
                    name,
                    PitCheckResult.Status.WARN,
                    String.format("Current low (%.2fA) - motor may be marginal", avgCurrent),
                    duration
                );
            }
        }

        // All checks passed
        String message = String.format("Encoder delta: %.4f", encoderDelta);
        if (hasCurrentSensor) {
            message += String.format(", Current: %.2fA", avgCurrent);
        }
        
        return new PitCheckResult(name, PitCheckResult.Status.PASS, message, duration);
    }

    @Override
    public void cleanup() {
        setMotorPower(0.0);
    }

    // Abstract methods to be implemented by concrete steps
    protected abstract void setMotorPower(double power);
    protected abstract Supplier<Double> getPositionSupplier();
    protected abstract Supplier<Double> getVelocitySupplier();
    protected abstract Supplier<Double> getCurrentSupplier();
    protected abstract Supplier<Double> getVoltageSupplier();
}
