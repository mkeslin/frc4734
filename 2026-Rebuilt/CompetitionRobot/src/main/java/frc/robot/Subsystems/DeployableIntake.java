package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIds.INTAKE_DEPLOY;
import static frc.robot.Constants.CANIds.INTAKE_MOTOR;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.MIN_DEPLOY_POSITION_FOR_INTAKE;
import static frc.robot.Constants.IntakeConstants.MOTION_MAGIC_ACCELERATION;
import static frc.robot.Constants.IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.MOTION_MAGIC_JERK;
import static frc.robot.Constants.IntakeConstants.kA;
import static frc.robot.Constants.IntakeConstants.kD;
import static frc.robot.Constants.IntakeConstants.kG;
import static frc.robot.Constants.IntakeConstants.kI;
import static frc.robot.Constants.IntakeConstants.kP;
import static frc.robot.Constants.IntakeConstants.kS;
import static frc.robot.Constants.IntakeConstants.kV;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.TelemetryCalcs;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Constants.IntakeConstants.IntakeSpeed;
import frc.robot.Subsystems.Bases.BaseDeployableIntake;

/**
 * DeployableIntake subsystem that controls the robot's deployable intake mechanism.
 * Uses two TalonFX motors:
 * - Deploy motor: Position control for deploy/stow (MotionMagic)
 * - Intake motor: Velocity control for intake on/off/forward/reverse
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Prevents intake from running when stowed (below minimum deploy position)</li>
 *   <li>Uses MotionMagic for smooth deploy/stow motion</li>
 *   <li>Publishes deploy position and intake speed to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseDeployableIntake
 * @see DeployPosition
 * @see IntakeSpeed
 * @see RobotState
 */
public class DeployableIntake extends SubsystemBase implements BaseDeployableIntake<DeployPosition, IntakeSpeed> {
    private final DoublePublisher deployPositionPub = TelemetryCalcs.createMechanismsPublisher("Intake Deploy Position");
    private final DoublePublisher intakeSpeedPub = TelemetryCalcs.createMechanismsPublisher("Intake Speed");

    private TalonFX m_deployMotor;
    private TalonFX m_intakeMotor;
    private final VoltageOut m_deployVoltReq = new VoltageOut(0.0);
    private final VoltageOut m_intakeVoltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_deploySysIdRoutine;
    private final SysIdRoutine m_intakeSysIdRoutine;

    private PositionTracker m_positionTracker;

    private MotionMagicVoltage m_deployRequest = new MotionMagicVoltage(0);

    private boolean initialized;

    public DeployableIntake() {
        // SysId routine for deploy motor
        m_deploySysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).div(Seconds.of(1)),
                        Volts.of(0.5),
                        null,
                        (state) -> SignalLogger.writeString("Intake Deploy State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_deployMotor.setControl(m_deployVoltReq.withOutput(volts.in(Volts))),
                        null,
                        this,
                        "IntakeDeploy"));

        // SysId routine for intake motor
        m_intakeSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).div(Seconds.of(1)),
                        Volts.of(0.5),
                        null,
                        (state) -> SignalLogger.writeString("Intake Motor State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_intakeMotor.setControl(m_intakeVoltReq.withOutput(volts.in(Volts))),
                        null,
                        this,
                        "IntakeMotor"));

        // Configure deploy motor (position control with MotionMagic)
        var deployConfigs = new TalonFXConfiguration();
        var deploySlot0 = deployConfigs.Slot0;
        deploySlot0.kG = kG;
        deploySlot0.kS = kS;
        deploySlot0.kV = kV;
        deploySlot0.kA = kA;
        deploySlot0.kP = kP;
        deploySlot0.kI = kI;
        deploySlot0.kD = kD;

        var deployMotionMagic = deployConfigs.MotionMagic;
        deployMotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        deployMotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        deployMotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        deployConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_deployMotor = new TalonFX(INTAKE_DEPLOY);
        m_deployMotor.setNeutralMode(NeutralModeValue.Brake);
        m_deployMotor.getConfigurator().apply(deployConfigs);
        CurrentLimitsConfigs deployCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(IntakeConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(IntakeConstants.STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(IntakeConstants.STATOR_CURRENT_LIMIT_ENABLE);
        ClosedLoopRampsConfigs deployRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(IntakeConstants.CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC);
        VoltageConfigs deployVoltage = new VoltageConfigs()
                .withPeakForwardVoltage(IntakeConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(IntakeConstants.PEAK_REVERSE_VOLTAGE);
        m_deployMotor.getConfigurator().apply(deployCurrentLimits);
        m_deployMotor.getConfigurator().apply(deployRamps);
        m_deployMotor.getConfigurator().apply(deployVoltage);

        // Configure intake motor (velocity control): Slot0, current limits, motor output, ramp, voltage
        m_intakeMotor = new TalonFX(INTAKE_MOTOR);
        m_intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        Slot0Configs intakeSlot0 = new Slot0Configs()
                .withKV(IntakeConstants.INTAKE_VELOCITY_KV).withKS(IntakeConstants.INTAKE_VELOCITY_KS)
                .withKP(IntakeConstants.INTAKE_VELOCITY_KP).withKI(IntakeConstants.INTAKE_VELOCITY_KI).withKD(IntakeConstants.INTAKE_VELOCITY_KD);
        CurrentLimitsConfigs intakeCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(IntakeConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(IntakeConstants.STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(IntakeConstants.STATOR_CURRENT_LIMIT_ENABLE);
        InvertedValue intakeInvert = IntakeConstants.INTAKE_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        MotorOutputConfigs intakeMotorOutput = new MotorOutputConfigs().withInverted(intakeInvert);
        ClosedLoopRampsConfigs intakeRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(IntakeConstants.CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC);
        VoltageConfigs intakeVoltage = new VoltageConfigs()
                .withPeakForwardVoltage(IntakeConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(IntakeConstants.PEAK_REVERSE_VOLTAGE);
        m_intakeMotor.getConfigurator().apply(intakeSlot0);
        m_intakeMotor.getConfigurator().apply(intakeCurrentLimits);
        m_intakeMotor.getConfigurator().apply(intakeMotorOutput);
        m_intakeMotor.getConfigurator().apply(intakeRamps);
        m_intakeMotor.getConfigurator().apply(intakeVoltage);

        resetDeployPosition();
        resetIntakeSpeed();
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation if needed
    }

    /**
     * Gets whether the intake has been initialized (positions reset).
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

    // Deploy position methods

    @Override
    public double getDeployPosition() {
        return m_deployMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void resetDeployPosition() {
        m_deployMotor.setPosition(DeployPosition.STOWED.value);
        initialized = true;
    }

    @Override
    public boolean isDeployed() {
        return getDeployPosition() >= MIN_DEPLOY_POSITION_FOR_INTAKE;
    }

    private Command moveToDeployPositionCommand(double goalPosition) {
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized()) {
                m_deployMotor.setControl(m_deployRequest.withPosition(goalPosition));
            } else {
                m_deployMotor.stopMotor();
            }
            if (m_positionTracker != null) {
                deployPositionPub.set(m_positionTracker.getIntakeDeployPosition());
            }
        })
                .until(() -> Math.abs(getDeployPosition() - goalPosition) < DEPLOY_TOLERANCE)
                .withName("deployableIntake.moveToDeployPosition");
    }

    @Override
    public Command moveToSetDeployPositionCommand(Supplier<DeployPosition> goalPositionSupplier) {
        Objects.requireNonNull(goalPositionSupplier, "goalPositionSupplier cannot be null");
        return Commands.sequence(
                moveToDeployPositionCommand(goalPositionSupplier.get().value))
                .withTimeout(3)
                .withName("deployableIntake.moveToSetDeployPosition");
    }

    @Override
    public Command moveToArbitraryDeployPositionCommand(Supplier<Double> goalPositionSupplier) {
        Objects.requireNonNull(goalPositionSupplier, "goalPositionSupplier cannot be null");
        return Commands.sequence(
                moveToDeployPositionCommand(goalPositionSupplier.get()))
                .withName("deployableIntake.moveToArbitraryDeployPosition");
    }

    @Override
    public Command moveDeployPositionDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return Commands.sequence(
                moveToDeployPositionCommand(getDeployPosition() + delta.get()))
                .withName("deployableIntake.moveDeployPositionDelta");
    }

    @Override
    public Command resetDeployPositionCommand() {
        return runOnce(this::resetDeployPosition).withName("deployableIntake.resetDeployPosition");
    }

    // Intake speed methods

    @Override
    public double getIntakeSpeed() {
        return m_intakeMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetIntakeSpeed() {
        m_intakeMotor.stopMotor();
    }

    private Command moveToIntakeSpeedCommand(double goalVelocity) {
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized()) {
                // Check if deployed before allowing intake
                if (isDeployed() || Math.abs(goalVelocity) < 0.01) {
                    VelocityVoltage velocityOut = new VelocityVoltage(0);
                    velocityOut.Slot = 0;
                    m_intakeMotor.setControl(velocityOut.withVelocity(goalVelocity));
                } else {
                    // Stop intake if not deployed
                    m_intakeMotor.stopMotor();
                }
            } else {
                m_intakeMotor.stopMotor();
            }
            if (m_positionTracker != null) {
                intakeSpeedPub.set(m_positionTracker.getIntakeSpeed());
            }
        })
                .withName("deployableIntake.moveToIntakeSpeed");
    }

    @Override
    public Command moveToSetIntakeSpeedCommand(Supplier<IntakeSpeed> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToIntakeSpeedCommand(goalSpeedSupplier.get().value))
                .withTimeout(3)
                .withName("deployableIntake.moveToSetIntakeSpeed");
    }

    @Override
    public Command moveToArbitraryIntakeSpeedCommand(Supplier<Double> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToIntakeSpeedCommand(goalSpeedSupplier.get()))
                .withName("deployableIntake.moveToArbitraryIntakeSpeed");
    }

    @Override
    public Command moveIntakeSpeedDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return Commands.sequence(
                moveToIntakeSpeedCommand(getIntakeSpeed() + delta.get()))
                .withName("deployableIntake.moveIntakeSpeedDelta");
    }

    @Override
    public Command resetIntakeSpeedCommand() {
        return runOnce(this::resetIntakeSpeed).withName("deployableIntake.resetIntakeSpeed");
    }

    // Coordination methods

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            m_deployMotor.stopMotor();
            m_intakeMotor.stopMotor();
        })
                .andThen(() -> {
                    m_deployMotor.setNeutralMode(NeutralModeValue.Coast);
                    m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("deployableIntake.coastMotorsCommand");
    }

    // SysId commands

    public Command sysIdDeployQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_deploySysIdRoutine.quasistatic(direction).withName("deployableIntake.sysIdDeployQuasistatic");
    }

    public Command sysIdDeployDynamicCommand(SysIdRoutine.Direction direction) {
        return m_deploySysIdRoutine.dynamic(direction).withName("deployableIntake.sysIdDeployDynamic");
    }

    public Command sysIdIntakeQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_intakeSysIdRoutine.quasistatic(direction).withName("deployableIntake.sysIdIntakeQuasistatic");
    }

    public Command sysIdIntakeDynamicCommand(SysIdRoutine.Direction direction) {
        return m_intakeSysIdRoutine.dynamic(direction).withName("deployableIntake.sysIdIntakeDynamic");
    }

    /**
     * Cleans up resources when the robot is disabled.
     * Stops both motors and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_deployMotor != null) {
            m_deployMotor.stopMotor();
        }
        if (m_intakeMotor != null) {
            m_intakeMotor.stopMotor();
        }
        if (deployPositionPub != null) {
            deployPositionPub.close();
        }
        if (intakeSpeedPub != null) {
            intakeSpeedPub.close();
        }
    }
}
