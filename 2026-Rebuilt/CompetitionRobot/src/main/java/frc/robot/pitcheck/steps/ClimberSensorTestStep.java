package frc.robot.pitcheck.steps;

import frc.robot.pitcheck.MotorPulseTestStep;
import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;

import java.util.function.Supplier;

/**
 * Climber sensor test - checks sensors and optionally pulses motor if allowed.
 */
public abstract class ClimberSensorTestStep extends MotorPulseTestStep implements PitCheckStep {
    private boolean motionAllowed;

    public ClimberSensorTestStep() {
        super(
            "Climber Sensor Test",
            PitCheckConstants.CLIMBER_PULSE_DURATION,
            PitCheckConstants.CLIMBER_PULSE_POWER,
            PitCheckConstants.DRIVE_MIN_ENCODER_DELTA,
            PitCheckConstants.CLIMBER_MIN_CURRENT,
            PitCheckConstants.CLIMBER_MAX_CURRENT
        );
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        motionAllowed = climberMotionAllowed;
        return true; // Can always check sensors
    }

    @Override
    public boolean initialize() {
        // First check sensors without motion
        return true;
    }

    @Override
    public void run(double elapsedTime) {
        if (motionAllowed && elapsedTime < PitCheckConstants.CLIMBER_PULSE_DURATION) {
            // Pulse motor
            super.run(elapsedTime);
        } else {
            // Just check sensors, no motion
            setMotorPower(0.0);
        }
    }

    @Override
    public boolean isComplete(double elapsedTime) {
        if (motionAllowed) {
            return super.isComplete(elapsedTime);
        } else {
            return elapsedTime > 0.2; // Quick sensor check
        }
    }

    @Override
    public PitCheckResult evaluate(double duration) {
        // Check sensors first
        PitCheckResult sensorResult = checkSensors(duration);
        if (sensorResult.isFailure()) {
            return sensorResult;
        }

        // If motion was allowed, evaluate motor pulse
        if (motionAllowed) {
            PitCheckResult motorResult = super.evaluate(duration);
            if (motorResult.isFailure()) {
                return motorResult;
            }
            if (motorResult.getStatus() == PitCheckResult.Status.WARN) {
                return motorResult;
            }
        }

        return sensorResult;
    }

    private PitCheckResult checkSensors(double duration) {
        // Check limit switches
        if (hasLimitSwitches()) {
            boolean lowerLimit = getLowerLimitSwitch();
            boolean upperLimit = getUpperLimitSwitch();
            
            // Both limits active is suspicious
            if (lowerLimit && upperLimit) {
                return new PitCheckResult(
                    getName(),
                    PitCheckResult.Status.WARN,
                    "Both limit switches active",
                    duration
                );
            }
        }

        // Check encoder present
        if (hasEncoder()) {
            double position = getClimberPositionSupplier().get();
            if (Double.isNaN(position)) {
                return new PitCheckResult(
                    getName(),
                    PitCheckResult.Status.FAIL,
                    "Encoder not responding",
                    duration
                );
            }
        }

        String message = "Sensors OK";
        if (motionAllowed) {
            message += ", motion tested";
        } else {
            message += " (motion skipped)";
        }

        return new PitCheckResult(
            getName(),
            PitCheckResult.Status.PASS,
            message,
            duration
        );
    }

    @Override
    protected void setMotorPower(double power) {
        setClimberPower(power);
    }

    @Override
    protected Supplier<Double> getPositionSupplier() {
        return getClimberPositionSupplier();
    }

    @Override
    protected Supplier<Double> getVelocitySupplier() {
        return getClimberVelocitySupplier();
    }

    @Override
    protected Supplier<Double> getCurrentSupplier() {
        return getClimberCurrentSupplier();
    }

    @Override
    protected Supplier<Double> getVoltageSupplier() {
        return getClimberVoltageSupplier();
    }

    // Abstract methods
    protected abstract void setClimberPower(double power);
    protected abstract Supplier<Double> getClimberPositionSupplier();
    protected abstract Supplier<Double> getClimberVelocitySupplier();
    protected abstract Supplier<Double> getClimberCurrentSupplier();
    protected abstract Supplier<Double> getClimberVoltageSupplier();
    
    protected abstract boolean hasLimitSwitches();
    protected abstract boolean getLowerLimitSwitch();
    protected abstract boolean getUpperLimitSwitch();
    protected abstract boolean hasEncoder();
}
