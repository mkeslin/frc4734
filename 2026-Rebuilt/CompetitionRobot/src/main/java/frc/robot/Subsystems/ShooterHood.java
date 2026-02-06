package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterHoodConstants.HoodPosition;
import static frc.robot.Constants.ShooterHoodConstants.PWM_CHANNEL;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.TelemetryCalcs;
import frc.robot.Subsystems.Bases.BaseSingleJointedArm;

/**
 * Shooter hood subsystem using an Actuonix L16-R 140mm linear servo on roboRIO PWM.
 * Standard hobby-servo pulse (1–2 ms); position 0 = retracted, 1 = extended. The actuator
 * holds position on its own; we only set the target (no position feedback to roboRIO).
 *
 * <p>Safety: position is not applied until robot is initialized via {@link RobotState}.
 */
public class ShooterHood extends SubsystemBase implements BaseSingleJointedArm<HoodPosition> {
    private final DoublePublisher hoodPositionPub = TelemetryCalcs.createMechanismsPublisher("Shooter Hood Position");

    private final Servo m_servo;
    /** Last commanded position (0–1); L16-R does not report position back. */
    private double m_setPosition;

    private boolean initialized;

    public ShooterHood() {
        m_servo = new Servo(PWM_CHANNEL);
        m_setPosition = HoodPosition.IN.value;
        initialized = true;
    }

    @Override
    public void simulationPeriodic() {
        // Add simulation if needed
    }

    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Last set position in [0, 1]. The L16-R does not provide feedback; this is command-only.
     */
    @Override
    public double getPosition() {
        return m_setPosition;
    }

    @Override
    public void resetPosition() {
        setPositionInternal(HoodPosition.IN.value);
        initialized = true;
    }

    /** Applies position only when robot is initialized; otherwise leaves servo at last position. */
    private void setPositionInternal(double position) {
        position = MathUtil.clamp(position, 0.0, 1.0);
        if (RobotState.getInstance().isInitialized()) {
            m_servo.setPosition(position);
            m_setPosition = position;
        }
    }

    @Override
    public Command moveToSetPositionCommand(Supplier<HoodPosition> goalPositionSupplier) {
        Objects.requireNonNull(goalPositionSupplier, "goalPositionSupplier cannot be null");
        return runOnce(() -> setPositionInternal(goalPositionSupplier.get().value))
                .withName("shooterHood.moveToSetPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        Objects.requireNonNull(goalPositionSupplier, "goalPositionSupplier cannot be null");
        return runOnce(() -> setPositionInternal(MathUtil.clamp(goalPositionSupplier.get(), 0.0, 1.0)))
                .withName("shooterHood.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return runOnce(() -> setPositionInternal(MathUtil.clamp(m_setPosition + delta.get(), 0.0, 1.0)))
                .withName("shooterHood.movePositionDelta");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("shooterHood.resetPosition");
    }

    @Override
    public Command coastMotorsCommand() {
        // Linear servo holds position; no motor to coast. Cancel any ongoing move by leaving at current setpoint.
        return runOnce(() -> { /* no-op; servo holds */ })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooterHood.coastMotorsCommand");
    }

    /**
     * Releases servo output (optional). Call from robot disabled/cleanup if desired.
     */
    public void cleanup() {
        if (m_servo != null) {
            m_servo.setDisabled();
        }
        if (hoodPositionPub != null) {
            hoodPositionPub.close();
        }
    }

    @Override
    public void periodic() {
        hoodPositionPub.set(m_setPosition);
    }
}
