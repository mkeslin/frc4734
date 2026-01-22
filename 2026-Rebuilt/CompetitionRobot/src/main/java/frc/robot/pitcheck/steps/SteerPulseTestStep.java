package frc.robot.pitcheck.steps;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;
import frc.robot.pitcheck.PitCheckTelemetry;

/**
 * Steer module pulse test - commands a small steering angle change.
 * 
 * This is an abstract base class. Subclasses must provide:
 * - A way to command steering angles
 * - Steer encoder position supplier
 */
public abstract class SteerPulseTestStep implements PitCheckStep {
    private double startTime;
    private double startAngle;
    private List<PitCheckTelemetry.Sample> samples;
    private boolean samplesCollected;

    @Override
    public String getName() {
        return "Steer Pulse Test";
    }

    @Override
    public double getTimeout() {
        return PitCheckConstants.STEER_PULSE_DURATION + 0.5;
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return robotOnBlocks; // Requires blocks
    }

    @Override
    public boolean initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        startAngle = getSteerAngleSupplier().get();
        samples = null;
        samplesCollected = false;
        return true;
    }

    @Override
    public void run(double elapsedTime) {
        if (elapsedTime < PitCheckConstants.STEER_PULSE_DURATION) {
            // Command small angle change
            double targetAngle = startAngle + PitCheckConstants.STEER_PULSE_ANGLE;
            commandSteerAngle(new Rotation2d(targetAngle));

            // Collect samples
            if (!samplesCollected) {
                samples = PitCheckTelemetry.collectSamples(
                    PitCheckConstants.STEER_PULSE_DURATION,
                    PitCheckConstants.TELEMETRY_SAMPLE_RATE,
                    getSteerAngleSupplier(),
                    () -> 0.0, // velocity not needed
                    getSteerCurrentSupplier(),
                    getVoltageSupplier()
                );
                samplesCollected = true;
            }
        } else {
            // Return to start angle
            commandSteerAngle(new Rotation2d(startAngle));
        }
    }

    @Override
    public boolean isComplete(double elapsedTime) {
        return elapsedTime >= PitCheckConstants.STEER_PULSE_DURATION + 0.1;
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

        double angleDelta = PitCheckTelemetry.calculateEncoderDelta(samples);

        if (angleDelta < PitCheckConstants.STEER_MIN_ENCODER_DELTA) {
            return new PitCheckResult(
                getName(),
                PitCheckResult.Status.FAIL,
                String.format("Steer encoder did not move (delta: %.4f rad)", angleDelta),
                duration
            );
        }

        return new PitCheckResult(
            getName(),
            PitCheckResult.Status.PASS,
            String.format("Steer angle delta: %.4f rad", angleDelta),
            duration
        );
    }

    @Override
    public void cleanup() {
        commandSteerAngle(new Rotation2d(startAngle));
    }

    // Abstract methods
    protected abstract void commandSteerAngle(Rotation2d angle);
    protected abstract Supplier<Double> getSteerAngleSupplier();
    protected abstract Supplier<Double> getSteerCurrentSupplier();
    protected abstract Supplier<Double> getVoltageSupplier();
}
