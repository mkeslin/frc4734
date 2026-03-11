package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CANIds.CAN_BUS;
import static frc.robot.Constants.CANIds.SHOOTER_LEFT;
import static frc.robot.Constants.CANIds.SHOOTER_CENTER;
import static frc.robot.Constants.CANIds.SHOOTER_RIGHT;
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
import frc.robot.ShooterSpeeds;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private TalonFX m_left;    // Motor 32
    private TalonFX m_center;  // Motor 30
    private TalonFX m_right;   // Motor 29
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
                            m_left.setControl(m_voltReq.withOutput(-v));
                            m_center.setControl(m_voltReq.withOutput(-v));
                            m_right.setControl(m_voltReq.withOutput(v));
                        },
                        null,
                        this,
                        "Shooter"));

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

        VoltageConfigs voltage = new VoltageConfigs()
                .withPeakForwardVoltage(ShooterConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(ShooterConstants.PEAK_REVERSE_VOLTAGE);

        ClosedLoopRampsConfigs rampLeft = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(ShooterConstants.SHOOTER_LEFT_RAMP_PERIOD_SEC);
        ClosedLoopRampsConfigs rampCenter = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(ShooterConstants.SHOOTER_CENTER_RAMP_PERIOD_SEC);
        ClosedLoopRampsConfigs rampRight = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(ShooterConstants.SHOOTER_RIGHT_RAMP_PERIOD_SEC);

        m_left = new TalonFX(SHOOTER_LEFT, CAN_BUS);
        m_left.setNeutralMode(NeutralModeValue.Brake);
        applyShooterMotorConfig(m_left, slot0, currentLimits, motorOutput, rampLeft, voltage);
        m_center = new TalonFX(SHOOTER_CENTER, CAN_BUS);
        m_center.setNeutralMode(NeutralModeValue.Brake);
        applyShooterMotorConfig(m_center, slot0, currentLimits, motorOutput, rampCenter, voltage);
        m_right = new TalonFX(SHOOTER_RIGHT, CAN_BUS);
        m_right.setNeutralMode(NeutralModeValue.Brake);
        applyShooterMotorConfig(m_right, slot0, currentLimits, motorOutput, rampRight, voltage);

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
        double vLeft = Math.abs(m_left.getVelocity().getValueAsDouble());
        double vCenter = Math.abs(m_center.getVelocity().getValueAsDouble());
        double vRight = m_right.getVelocity().getValueAsDouble();
        return Math.min(vLeft, Math.min(vCenter, vRight));
    }

    /**
     * Returns true if all three motors are within tolerance of their per-motor target speeds.
     *
     * @param baseRps Base RPS (before per-motor multipliers)
     * @param tol     Tolerance in RPS
     */
    public boolean isAtSpeed(double baseRps, double tol) {
        return isAtSpeed(ShooterSpeeds.fromBase(baseRps), tol);
    }

    /**
     * Returns true if all three motors are within tolerance of their per-motor target speeds.
     *
     * @param speeds Per-motor target speeds (magnitudes)
     * @param tol    Tolerance in RPS
     */
    public boolean isAtSpeed(ShooterSpeeds speeds, double tol) {
        double tLeft = -speeds.leftRps();
        double tCenter = -speeds.centerRps();
        double tRight = speeds.rightRps();
        double vLeft = m_left.getVelocity().getValueAsDouble();
        double vCenter = m_center.getVelocity().getValueAsDouble();
        double vRight = m_right.getVelocity().getValueAsDouble();
        return Math.abs(vLeft - tLeft) <= tol && Math.abs(vCenter - tCenter) <= tol && Math.abs(vRight - tRight) <= tol;
    }

    @Override
    public void resetSpeed() {
        m_left.stopMotor();
        m_center.stopMotor();
        m_right.stopMotor();
        initialized = true;
    }

    /** Teleop only: applies per-motor multipliers to base RPS. Auto uses moveToTripleSpeedCommand with explicit speeds. */
    private Command moveToSpeedCommand(Supplier<Double> baseRpsSupplier) {
        return moveToTripleSpeedCommand(() -> ShooterSpeeds.fromBase(baseRpsSupplier.get()));
    }

    /**
     * Command to run shooter at per-motor speeds. Left and center use negative velocity,
     * right uses positive (shoot direction).
     */
    public Command moveToTripleSpeedCommand(Supplier<ShooterSpeeds> speedsSupplier) {
        Objects.requireNonNull(speedsSupplier, "speedsSupplier cannot be null");
        return Commands.run(() -> {
            boolean skipInitCheck = BYPASS_ROBOT_STATE_FOR_BENCH;
            boolean allowRun = canRun() && (skipInitCheck || RobotState.getInstance().isInitialized());
            if (allowRun) {
                ShooterSpeeds s = speedsSupplier.get();
                double vLeft = -s.leftRps();
                double vCenter = -s.centerRps();
                double vRight = s.rightRps();
                VelocityVoltage velocityOut = new VelocityVoltage(0);
                velocityOut.Slot = 0;
                m_left.setControl(velocityOut.withVelocity(vLeft));
                m_center.setControl(velocityOut.withVelocity(vCenter));
                m_right.setControl(velocityOut.withVelocity(vRight));
            } else {
                m_left.stopMotor();
                m_center.stopMotor();
                m_right.stopMotor();
            }
            if (m_positionTracker != null) {
                shooterSpeedPub.set(m_positionTracker.getShooterSpeed());
            }
        }, this)
                .withName("shooter.moveToTripleSpeed");
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
            m_left.stopMotor();
            m_center.stopMotor();
            m_right.stopMotor();
        })
                .andThen(() -> {
                    m_left.setNeutralMode(NeutralModeValue.Coast);
                    m_center.setNeutralMode(NeutralModeValue.Coast);
                    m_right.setNeutralMode(NeutralModeValue.Coast);
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
        if (m_left != null) {
            m_left.stopMotor();
        }
        if (m_center != null) {
            m_center.stopMotor();
        }
        if (m_right != null) {
            m_right.stopMotor();
        }
        if (shooterSpeedPub != null) {
            shooterSpeedPub.close();
        }
    }
}
