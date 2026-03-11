package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CANIds.CAN_BUS;
import static frc.robot.Constants.CANIds.SHOOTER_1;
import static frc.robot.Constants.CANIds.SHOOTER_2;
import static frc.robot.Constants.CANIds.SHOOTER_3;
import static edu.wpi.first.units.Units.Amps;
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

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.TelemetryCalcs;
import frc.robot.Constants.ShooterConstants.ShooterSpeed;
import frc.robot.Subsystems.Bases.BaseIntake;

/**
 * Shooter subsystem that controls the robot's shooter mechanism.
 * Uses three independently controlled TalonFX motors (one per axle) with velocity control.
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Extensible safety coordination hooks for future conditions</li>
 *   <li>Publishes shooter speed to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseIntake
 * @see ShooterSpeed
 * @see RobotState
 */
public class Shooter extends SubsystemBase implements BaseIntake<ShooterSpeed> {
    /** When true, shooter runs without requiring RobotState initialization (for bench troubleshooting only). */
    private static final boolean BYPASS_ROBOT_STATE_FOR_BENCH = true;

    private final DoublePublisher shooterSpeedPub = TelemetryCalcs.createMechanismsPublisher("Shooter Speed");

    private TalonFX m_axle1;
    private TalonFX m_axle2;
    private TalonFX m_axle3;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutine;

    private PositionTracker m_positionTracker;

    private boolean initialized;

    public Shooter() {
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).div(Seconds.of(1)),
                        Volts.of(0.5),
                        null,
                        (state) -> SignalLogger.writeString("Shooter State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            double v = volts.in(Volts);
                            m_axle1.setControl(m_voltReq.withOutput(v));
                            m_axle2.setControl(m_voltReq.withOutput(-v));
                            m_axle3.setControl(m_voltReq.withOutput(-v));
                        },
                        null,
                        this,
                        "Shooter"));

        m_axle1 = new TalonFX(SHOOTER_1, CAN_BUS);
        m_axle1.setNeutralMode(NeutralModeValue.Brake);

        // Velocity closed-loop Slot0 and current limits in one config
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kV = ShooterConstants.VELOCITY_KV;
        slot0.kS = ShooterConstants.VELOCITY_KS;
        slot0.kA = ShooterConstants.VELOCITY_KA;
        slot0.kP = ShooterConstants.VELOCITY_KP;
        slot0.kI = ShooterConstants.VELOCITY_KI;
        slot0.kD = ShooterConstants.VELOCITY_KD;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(ShooterConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(ShooterConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(ShooterConstants.STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(ShooterConstants.STATOR_CURRENT_LIMIT_ENABLE);

        InvertedValue invert = ShooterConstants.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        MotorOutputConfigs motorOutput = new MotorOutputConfigs().withInverted(invert);

        ClosedLoopRampsConfigs closedLoopRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(ShooterConstants.CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC);

        VoltageConfigs voltage = new VoltageConfigs()
                .withPeakForwardVoltage(ShooterConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(ShooterConstants.PEAK_REVERSE_VOLTAGE);

        applyShooterMotorConfig(m_axle1, slot0, currentLimits, motorOutput, closedLoopRamps, voltage);

        m_axle2 = new TalonFX(SHOOTER_2, CAN_BUS);
        m_axle2.setNeutralMode(NeutralModeValue.Brake);
        applyShooterMotorConfig(m_axle2, slot0, currentLimits, motorOutput, closedLoopRamps, voltage);

        m_axle3 = new TalonFX(SHOOTER_3, CAN_BUS);
        m_axle3.setNeutralMode(NeutralModeValue.Brake);
        applyShooterMotorConfig(m_axle3, slot0, currentLimits, motorOutput, closedLoopRamps, voltage);

        resetSpeed();
    }

    /** Applies Slot0, current limits, motor output, closed-loop ramps, and voltage config to a shooter Talon FX. */
    private static void applyShooterMotorConfig(TalonFX motor,
            Slot0Configs slot0, CurrentLimitsConfigs currentLimits, MotorOutputConfigs motorOutput,
            ClosedLoopRampsConfigs closedLoopRamps, VoltageConfigs voltage) {
        motor.getConfigurator().apply(slot0);
        motor.getConfigurator().apply(currentLimits);
        motor.getConfigurator().apply(motorOutput);
        motor.getConfigurator().apply(closedLoopRamps);
        motor.getConfigurator().apply(voltage);
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation if needed
    }

    /**
     * Gets whether the shooter has been initialized (speed reset).
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
     * Checks if the shooter can run based on safety conditions.
     * This is a placeholder method that can be extended with actual safety checks.
     * 
     * @return true if the shooter can run, false otherwise
     */
    private boolean canRun() {
        // TODO: Add safety coordination checks when conditions are determined
        // Examples:
        // - Only run when ready to shoot
        // - Coordinate with Feeder mechanism
        // - Check for ball presence before shooting
        return true;
    }

    @Override
    public double getSpeed() {
        double v1 = m_axle1.getVelocity().getValueAsDouble();
        double v2 = Math.abs(m_axle2.getVelocity().getValueAsDouble());
        double v3 = Math.abs(m_axle3.getVelocity().getValueAsDouble());
        return Math.min(v1, Math.min(v2, v3));
    }

    /**
     * Returns true if all three axles are within tolerance of their per-axle target speeds.
     *
     * @param baseRps Base RPS (before per-axle multipliers)
     * @param tol     Tolerance in RPS
     */
    public boolean isAtSpeed(double baseRps, double tol) {
        double t1 = baseRps * ShooterConstants.AXLE_1_SPEED_MULTIPLIER;
        double t2 = -baseRps * ShooterConstants.AXLE_2_SPEED_MULTIPLIER;
        double t3 = -baseRps * ShooterConstants.AXLE_3_SPEED_MULTIPLIER;
        double v1 = m_axle1.getVelocity().getValueAsDouble();
        double v2 = m_axle2.getVelocity().getValueAsDouble();
        double v3 = m_axle3.getVelocity().getValueAsDouble();
        return Math.abs(v1 - t1) <= tol && Math.abs(v2 - t2) <= tol && Math.abs(v3 - t3) <= tol;
    }

    @Override
    public void resetSpeed() {
        m_axle1.stopMotor();
        m_axle2.stopMotor();
        m_axle3.stopMotor();
        initialized = true;
    }

    private Command moveToSpeedCommand(Supplier<Double> baseRpsSupplier) {
        return run(() -> {
            boolean skipInitCheck = BYPASS_ROBOT_STATE_FOR_BENCH;
            boolean allowRun = canRun() && (skipInitCheck || RobotState.getInstance().isInitialized());
            if (allowRun) {
                double base = baseRpsSupplier.get();
                double v1 = base * ShooterConstants.AXLE_1_SPEED_MULTIPLIER;
                double v2 = -base * ShooterConstants.AXLE_2_SPEED_MULTIPLIER;
                double v3 = -base * ShooterConstants.AXLE_3_SPEED_MULTIPLIER;
                VelocityVoltage velocityOut = new VelocityVoltage(0);
                velocityOut.Slot = 0;
                m_axle1.setControl(velocityOut.withVelocity(v1));
                m_axle2.setControl(velocityOut.withVelocity(v2));
                m_axle3.setControl(velocityOut.withVelocity(v3));
            } else {
                m_axle1.stopMotor();
                m_axle2.stopMotor();
                m_axle3.stopMotor();
            }
            if (m_positionTracker != null) {
                shooterSpeedPub.set(m_positionTracker.getShooterSpeed());
            }
        })
                .withName("shooter.moveToSpeed");
    }

    @Override
    public Command moveToSetSpeedCommand(Supplier<ShooterSpeed> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return moveToSpeedCommand(() -> goalSpeedSupplier.get().value)
                .withTimeout(3)
                .withName("shooter.moveToSetSpeed");
    }

    @Override
    public Command moveToArbitrarySpeedCommand(Supplier<Double> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return moveToSpeedCommand(goalSpeedSupplier)
                .withName("shooter.moveToArbitrarySpeed");
    }

    @Override
    public Command moveSpeedDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return moveToSpeedCommand(() -> getSpeed() + delta.get())
                .withName("shooter.moveSpeedDelta");
    }

    @Override
    public Command resetSpeedCommand() {
        return runOnce(this::resetSpeed).withName("shooter.resetSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            m_axle1.stopMotor();
            m_axle2.stopMotor();
            m_axle3.stopMotor();
        })
                .andThen(() -> {
                    m_axle1.setNeutralMode(NeutralModeValue.Coast);
                    m_axle2.setNeutralMode(NeutralModeValue.Coast);
                    m_axle3.setNeutralMode(NeutralModeValue.Coast);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooter.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
    }

    /**
     * Cleans up resources when the robot is disabled.
     * Stops all motors and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_axle1 != null) {
            m_axle1.stopMotor();
        }
        if (m_axle2 != null) {
            m_axle2.stopMotor();
        }
        if (m_axle3 != null) {
            m_axle3.stopMotor();
        }
        if (shooterSpeedPub != null) {
            shooterSpeedPub.close();
        }
    }
}
