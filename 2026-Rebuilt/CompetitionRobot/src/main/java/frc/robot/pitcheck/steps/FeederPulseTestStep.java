package frc.robot.pitcheck.steps;

import frc.robot.pitcheck.MotorPulseTestStep;
import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckStep;

import java.util.function.Supplier;

/**
 * Feeder pulse test - pulses feeder motor and verifies operation.
 */
public abstract class FeederPulseTestStep extends MotorPulseTestStep implements PitCheckStep {
    public FeederPulseTestStep() {
        super(
            "Feeder Pulse Test",
            PitCheckConstants.FEEDER_PULSE_DURATION,
            PitCheckConstants.FEEDER_PULSE_POWER,
            PitCheckConstants.DRIVE_MIN_ENCODER_DELTA,
            PitCheckConstants.FEEDER_MIN_CURRENT,
            PitCheckConstants.FEEDER_MAX_CURRENT
        );
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return true;
    }

    @Override
    protected void setMotorPower(double power) {
        setFeederPower(power);
    }

    @Override
    protected Supplier<Double> getPositionSupplier() {
        return getFeederPositionSupplier();
    }

    @Override
    protected Supplier<Double> getVelocitySupplier() {
        return getFeederVelocitySupplier();
    }

    @Override
    protected Supplier<Double> getCurrentSupplier() {
        return getFeederCurrentSupplier();
    }

    @Override
    protected Supplier<Double> getVoltageSupplier() {
        return getFeederVoltageSupplier();
    }

    // Abstract methods
    protected abstract void setFeederPower(double power);
    protected abstract Supplier<Double> getFeederPositionSupplier();
    protected abstract Supplier<Double> getFeederVelocitySupplier();
    protected abstract Supplier<Double> getFeederCurrentSupplier();
    protected abstract Supplier<Double> getFeederVoltageSupplier();
}
