package frc.robot.pitcheck.steps;

import frc.robot.pitcheck.MotorPulseTestStep;
import frc.robot.pitcheck.PitCheckConstants;
import frc.robot.pitcheck.PitCheckResult;
import frc.robot.pitcheck.PitCheckStep;

import java.util.function.Supplier;

/**
 * Intake pulse test - pulses intake motor and verifies operation.
 * 
 * This is an abstract base class. Subclasses must provide motor control
 * and sensor suppliers.
 */
public abstract class IntakePulseTestStep extends MotorPulseTestStep implements PitCheckStep {
    public IntakePulseTestStep() {
        super(
            "Intake Pulse Test",
            PitCheckConstants.INTAKE_PULSE_DURATION,
            PitCheckConstants.INTAKE_PULSE_POWER,
            PitCheckConstants.DRIVE_MIN_ENCODER_DELTA, // Reuse drive threshold
            PitCheckConstants.INTAKE_MIN_CURRENT,
            PitCheckConstants.INTAKE_MAX_CURRENT
        );
    }

    @Override
    public boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed) {
        return true; // Intake doesn't require blocks
    }

    @Override
    protected void setMotorPower(double power) {
        setIntakePower(power);
    }

    @Override
    protected Supplier<Double> getPositionSupplier() {
        return getIntakePositionSupplier();
    }

    @Override
    protected Supplier<Double> getVelocitySupplier() {
        return getIntakeVelocitySupplier();
    }

    @Override
    protected Supplier<Double> getCurrentSupplier() {
        return getIntakeCurrentSupplier();
    }

    @Override
    protected Supplier<Double> getVoltageSupplier() {
        return getIntakeVoltageSupplier();
    }

    // Abstract methods to be implemented
    protected abstract void setIntakePower(double power);
    protected abstract Supplier<Double> getIntakePositionSupplier();
    protected abstract Supplier<Double> getIntakeVelocitySupplier();
    protected abstract Supplier<Double> getIntakeCurrentSupplier();
    protected abstract Supplier<Double> getIntakeVoltageSupplier();
}
