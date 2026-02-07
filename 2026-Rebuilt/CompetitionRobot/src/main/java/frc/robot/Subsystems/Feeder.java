package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CANIds.FEEDER;
import static edu.wpi.first.units.Units.Volts;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.FeederConstants;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.TelemetryCalcs;
import frc.robot.Constants.FeederConstants.FeederSpeed;
import frc.robot.Subsystems.Bases.BaseIntake;

/**
 * Feeder subsystem that controls the robot's feeder mechanism.
 * Uses a TalonFX motor with velocity control to feed balls from the back of the hopper to the shooter.
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Extensible safety coordination hooks for future conditions</li>
 *   <li>Publishes feeder speed to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseIntake
 * @see FeederSpeed
 * @see RobotState
 */
public class Feeder extends SubsystemBase implements BaseIntake<FeederSpeed> {
    private final DoublePublisher feederSpeedPub = TelemetryCalcs.createMechanismsPublisher("Feeder Speed");

    private TalonFX m_feederMotor;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutine;

    private PositionTracker m_positionTracker;

    private boolean initialized;

    public Feeder() {
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).div(Seconds.of(1)),
                        Volts.of(0.5),
                        null,
                        (state) -> SignalLogger.writeString("Feeder State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_feederMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null,
                        this,
                        "Feeder"));

        m_feederMotor = new TalonFX(FEEDER);
        m_feederMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0 = new Slot0Configs()
                .withKV(FeederConstants.VELOCITY_KV).withKS(FeederConstants.VELOCITY_KS)
                .withKP(FeederConstants.VELOCITY_KP).withKI(FeederConstants.VELOCITY_KI).withKD(FeederConstants.VELOCITY_KD);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(FeederConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(FeederConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(FeederConstants.STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(FeederConstants.STATOR_CURRENT_LIMIT_ENABLE);
        InvertedValue invert = FeederConstants.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        MotorOutputConfigs motorOutput = new MotorOutputConfigs().withInverted(invert);
        ClosedLoopRampsConfigs closedLoopRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(FeederConstants.CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC);
        VoltageConfigs voltage = new VoltageConfigs()
                .withPeakForwardVoltage(FeederConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(FeederConstants.PEAK_REVERSE_VOLTAGE);

        m_feederMotor.getConfigurator().apply(slot0);
        m_feederMotor.getConfigurator().apply(currentLimits);
        m_feederMotor.getConfigurator().apply(motorOutput);
        m_feederMotor.getConfigurator().apply(closedLoopRamps);
        m_feederMotor.getConfigurator().apply(voltage);

        resetSpeed();
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation if needed
    }

    /**
     * Gets whether the feeder has been initialized (speed reset).
     * 
     * @return true if initialized, false otherwise
     */
    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Updates the PositionTracker reference. Used during initialization to ensure
     * all subsystems share the same PositionTracker instance with real suppliers.
     * 
     * @param positionTracker The PositionTracker instance to use (can be null)
     */
    public void setPositionTracker(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
    }

    /**
     * Checks if the feeder can run based on safety conditions.
     * This is a placeholder method that can be extended with actual safety checks.
     * 
     * @return true if the feeder can run, false otherwise
     */
    private boolean canRun() {
        // TODO: Add safety coordination checks when conditions are determined
        // Examples:
        // - Only run when shooter is ready
        // - Coordinate with Floor conveyor
        // - Prevent running when hopper is empty
        return true;
    }

    @Override
    public double getSpeed() {
        return m_feederMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetSpeed() {
        m_feederMotor.stopMotor();
        initialized = true;
    }

    private Command moveToSpeedCommand(double goalVelocity) {
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized() && canRun()) {
                VelocityVoltage velocityOut = new VelocityVoltage(0);
                velocityOut.Slot = 0;
                m_feederMotor.setControl(velocityOut.withVelocity(goalVelocity));
            } else {
                m_feederMotor.stopMotor();
            }
            if (m_positionTracker != null) {
                feederSpeedPub.set(m_positionTracker.getFeederSpeed());
            }
        })
                .withName("feeder.moveToSpeed");
    }

    @Override
    public Command moveToSetSpeedCommand(Supplier<FeederSpeed> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get().value))
                .withTimeout(3)
                .withName("feeder.moveToSetSpeed");
    }

    @Override
    public Command moveToArbitrarySpeedCommand(Supplier<Double> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get()))
                .withName("feeder.moveToArbitrarySpeed");
    }

    @Override
    public Command moveSpeedDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(getSpeed() + delta.get()))
                .withName("feeder.moveSpeedDelta");
    }

    @Override
    public Command resetSpeedCommand() {
        return runOnce(this::resetSpeed).withName("feeder.resetSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> m_feederMotor.stopMotor())
                .andThen(() -> m_feederMotor.setNeutralMode(NeutralModeValue.Coast))
                .finallyDo((d) -> {
                    // Reset to brake mode if needed
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("feeder.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("feeder.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("feeder.sysIdDynamic");
    }

    /**
     * Cleans up resources when the robot is disabled.
     * Stops the motor and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_feederMotor != null) {
            m_feederMotor.stopMotor();
        }
        if (feederSpeedPub != null) {
            feederSpeedPub.close();
        }
    }
}
