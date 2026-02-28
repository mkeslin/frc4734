package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CANIds.CAN_BUS;
import static frc.robot.Constants.CANIds.FLOOR_CONVEYOR;
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

import frc.robot.Constants.FloorConstants;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.TelemetryCalcs;
import frc.robot.Constants.FloorConstants.ConveyorSpeed;
import frc.robot.Subsystems.Bases.BaseIntake;

/**
 * Floor subsystem that controls the robot's floor conveyor belt mechanism.
 * Uses a TalonFX motor with velocity control to move balls from the hopper toward the feeder/shooter.
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Extensible safety coordination hooks for future conditions</li>
 *   <li>Publishes floor speed to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseIntake
 * @see ConveyorSpeed
 * @see RobotState
 */
public class Floor extends SubsystemBase implements BaseIntake<ConveyorSpeed> {
    private final DoublePublisher floorSpeedPub = TelemetryCalcs.createMechanismsPublisher("Floor Speed");

    private TalonFX m_floorMotor;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutine;

    private PositionTracker m_positionTracker;

    private boolean initialized;

    public Floor() {
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).div(Seconds.of(1)),
                        Volts.of(0.5),
                        null,
                        (state) -> SignalLogger.writeString("Floor State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_floorMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null,
                        this,
                        "Floor"));

        m_floorMotor = new TalonFX(FLOOR_CONVEYOR, CAN_BUS);
        m_floorMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0 = new Slot0Configs()
                .withKV(FloorConstants.VELOCITY_KV).withKS(FloorConstants.VELOCITY_KS)
                .withKP(FloorConstants.VELOCITY_KP).withKI(FloorConstants.VELOCITY_KI).withKD(FloorConstants.VELOCITY_KD);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(FloorConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(FloorConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(FloorConstants.STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(FloorConstants.STATOR_CURRENT_LIMIT_ENABLE);
        InvertedValue invert = FloorConstants.MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        MotorOutputConfigs motorOutput = new MotorOutputConfigs().withInverted(invert);
        ClosedLoopRampsConfigs closedLoopRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(FloorConstants.CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC);
        VoltageConfigs voltage = new VoltageConfigs()
                .withPeakForwardVoltage(FloorConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(FloorConstants.PEAK_REVERSE_VOLTAGE);

        m_floorMotor.getConfigurator().apply(slot0);
        m_floorMotor.getConfigurator().apply(currentLimits);
        m_floorMotor.getConfigurator().apply(motorOutput);
        m_floorMotor.getConfigurator().apply(closedLoopRamps);
        m_floorMotor.getConfigurator().apply(voltage);

        resetSpeed();
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation if needed
    }

    /**
     * Gets whether the floor has been initialized (speed reset).
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
     * Checks if the floor conveyor can run based on safety conditions.
     * This is a placeholder method that can be extended with actual safety checks.
     * 
     * @return true if the conveyor can run, false otherwise
     */
    private boolean canRun() {
        // TODO: Add safety coordination checks when conditions are determined
        // Examples:
        // - Only run when shooter is ready
        // - Stop when hopper is full
        // - Coordinate with feeder mechanism
        return true;
    }

    @Override
    public double getSpeed() {
        return m_floorMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetSpeed() {
        m_floorMotor.stopMotor();
        initialized = true;
    }

    private Command moveToSpeedCommand(double goalVelocity) {
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized() && canRun()) {
                VelocityVoltage velocityOut = new VelocityVoltage(0);
                velocityOut.Slot = 0;
                m_floorMotor.setControl(velocityOut.withVelocity(goalVelocity));
            } else {
                m_floorMotor.stopMotor();
            }
            if (m_positionTracker != null) {
                floorSpeedPub.set(m_positionTracker.getFloorSpeed());
            }
        })
                .withName("floor.moveToSpeed");
    }

    @Override
    public Command moveToSetSpeedCommand(Supplier<ConveyorSpeed> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get().value))
                .withTimeout(3)
                .withName("floor.moveToSetSpeed");
    }

    @Override
    public Command moveToArbitrarySpeedCommand(Supplier<Double> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get()))
                .withName("floor.moveToArbitrarySpeed");
    }

    @Override
    public Command moveSpeedDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(getSpeed() + delta.get()))
                .withName("floor.moveSpeedDelta");
    }

    @Override
    public Command resetSpeedCommand() {
        return runOnce(this::resetSpeed).withName("floor.resetSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> m_floorMotor.stopMotor())
                .andThen(() -> m_floorMotor.setNeutralMode(NeutralModeValue.Coast))
                .finallyDo((d) -> {
                    // Reset to brake mode if needed
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("floor.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("floor.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("floor.sysIdDynamic");
    }

    /**
     * Cleans up resources when the robot is disabled.
     * Stops the motor and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_floorMotor != null) {
            m_floorMotor.stopMotor();
        }
        if (floorSpeedPub != null) {
            floorSpeedPub.close();
        }
    }
}
