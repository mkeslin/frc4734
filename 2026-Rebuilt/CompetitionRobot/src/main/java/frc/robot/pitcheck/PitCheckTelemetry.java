package frc.robot.pitcheck;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper utilities for reading telemetry during pit check tests.
 * 
 * Provides methods to sample motor current, encoder positions, and other sensor values.
 */
public final class PitCheckTelemetry {
    private PitCheckTelemetry() {
        // Utility class
    }

    /**
     * Represents a single telemetry sample.
     */
    public static class Sample {
        public final double timestamp;
        public final double encoderPosition;
        public final double encoderVelocity;
        public final double current;
        public final double voltage;

        public Sample(double timestamp, double encoderPosition, double encoderVelocity, 
                     double current, double voltage) {
            this.timestamp = timestamp;
            this.encoderPosition = encoderPosition;
            this.encoderVelocity = encoderVelocity;
            this.current = current;
            this.voltage = voltage;
        }
    }

    /**
     * Collects telemetry samples over a duration.
     * 
     * @param duration How long to sample (seconds)
     * @param sampleRate Sample rate (Hz)
     * @param positionSupplier Function to get current encoder position
     * @param velocitySupplier Function to get current encoder velocity
     * @param currentSupplier Function to get current motor current (amps)
     * @param voltageSupplier Function to get supply voltage (volts)
     * @return List of samples
     */
    public static List<Sample> collectSamples(
            double duration,
            double sampleRate,
            java.util.function.Supplier<Double> positionSupplier,
            java.util.function.Supplier<Double> velocitySupplier,
            java.util.function.Supplier<Double> currentSupplier,
            java.util.function.Supplier<Double> voltageSupplier) {
        
        List<Sample> samples = new ArrayList<>();
        double interval = 1.0 / sampleRate;
        int numSamples = (int) Math.ceil(duration * sampleRate);
        double startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        for (int i = 0; i < numSamples; i++) {
            double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;
            
            double position = positionSupplier != null ? positionSupplier.get() : 0.0;
            double velocity = velocitySupplier != null ? velocitySupplier.get() : 0.0;
            double current = currentSupplier != null ? currentSupplier.get() : 0.0;
            double voltage = voltageSupplier != null ? voltageSupplier.get() : 12.0;

            samples.add(new Sample(timestamp, position, velocity, current, voltage));

            // Wait for next sample interval
            edu.wpi.first.wpilibj.Timer.delay(interval);
        }

        return samples;
    }

    /**
     * Calculates encoder delta from samples.
     */
    public static double calculateEncoderDelta(List<Sample> samples) {
        if (samples.isEmpty()) {
            return 0.0;
        }
        double start = samples.get(0).encoderPosition;
        double end = samples.get(samples.size() - 1).encoderPosition;
        return Math.abs(end - start);
    }

    /**
     * Calculates average current from samples.
     */
    public static double calculateAverageCurrent(List<Sample> samples) {
        if (samples.isEmpty()) {
            return 0.0;
        }
        return samples.stream()
                .mapToDouble(s -> s.current)
                .average()
                .orElse(0.0);
    }

    /**
     * Calculates peak current from samples.
     */
    public static double calculatePeakCurrent(List<Sample> samples) {
        if (samples.isEmpty()) {
            return 0.0;
        }
        return samples.stream()
                .mapToDouble(s -> s.current)
                .max()
                .orElse(0.0);
    }

    /**
     * Calculates average voltage from samples.
     */
    public static double calculateAverageVoltage(List<Sample> samples) {
        if (samples.isEmpty()) {
            return 12.0;
        }
        return samples.stream()
                .mapToDouble(s -> s.voltage)
                .average()
                .orElse(12.0);
    }
}
