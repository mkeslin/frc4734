package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIds.ALGAE_INTAKE;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.Telemetry;
import frc.robot.Constants.AlgaeIntakeConstants.AlgaeIntakeSpeed;
import frc.robot.Subsystems.Bases.BaseIntake;

/**
 * AlgaeIntake subsystem that controls the robot's algae intake mechanism.
 * Uses a TalonFX motor with velocity control for speed-based operation.
 * The intake can run at predefined speeds (via AlgaeIntakeSpeed enum) or arbitrary speeds.
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Uses velocity control for smooth speed regulation</li>
 *   <li>Publishes intake speed to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseIntake
 * @see AlgaeIntakeSpeed
 * @see RobotState
 */
// @LoggedObject
public class AlgaeIntake extends SubsystemBase implements BaseIntake<AlgaeIntakeSpeed> {
    private final DoublePublisher algaeIntakePub = Telemetry.createMechanismsPublisher("Intake Speed");

    private TalonFX m_algaeIntakeMotor;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    // private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    // MOTOR_GEARBOX_REPR,
    // GEARING,
    // MOI,
    // COM_DISTANCE_METERS,
    // MIN_ANGLE_RADIANS,
    // MAX_ANGLE_RADIANS,
    // true,
    // ArmPosition.TOP.value);

    // private double simVelocity = 0.0;

    // private final MutVoltage sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    // private final MutAngle sysidPositionMeasure = MutableMeasure.mutable(Radians.of(0));
    // private final MutAngularVelocity sysidVelocityMeasure =
    // MutableMeasure.mutable(RadiansPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

    private PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;
    // private final Supplier<Pose3d> carriagePoseSupplier;

    private MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    private boolean initialized;

    public AlgaeIntake() {

        // this.carriagePoseSupplier = carriagePoseSupplier;

        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).div(Seconds.of(1)),
                        Volts.of(0.5),
                        null,
                        (state) -> SignalLogger.writeString("Algae State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_algaeIntakeMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null,
                        this,
                        "Algae"));

        // setDefaultCommand(moveToCurrentGoalCommand());

        // var talonFxConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        // var slot0Configs = talonFxConfigs.Slot0;
        // slot0Configs.kS = 0.25;
        // slot0Configs.kV = 0.12;
        // slot0Configs.kA = 0.01;
        // slot0Configs.kP = 3.8;
        // slot0Configs.kI = 0.0;
        // slot0Configs.kD = 0.1;

        // slot0Configs.kG = 0.25652;
        // slot0Configs.kS = 0.0;
        // slot0Configs.kV = 0.1039;
        // slot0Configs.kA = 0.11164;
        // slot0Configs.kP = 46.911;
        // slot0Configs.kI = 0.0;
        // slot0Configs.kD = 3.1607;
        // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        // // slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        // set Motion Magic settings
        // var motionMagicConfigs = talonFxConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_algaeIntakeMotor = new TalonFX(ALGAE_INTAKE);
        // m_algaeIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        // talonFxConfigs.CurrentLimits = new CurrentLimitsConfigs();
        // talonFxConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        // m_algaeIntakeMotor.getConfigurator().apply(talonFxConfigs);

        resetSpeed();
    }

    @Override
    public void simulationPeriodic() {
        // armSim.setInput(motor.getAppliedOutput());
        // armSim.update(0.020);
        // motor.getEncoder().setPosition(armSim.getAngleRads());
        // simVelocity = armSim.getVelocityRadPerSec();
        // ligament.setAngle(Units.radiansToDegrees(getPosition()) + 270);
    }

    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Updates the PositionTracker reference. Used during initialization to ensure
     * all subsystems share the same PositionTracker instance with real suppliers.
     */
    public void setPositionTracker(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
    }

    // // @Log(groups = "components")
    // public Pose3d getArmComponentPose() {
    // return carriagePoseSupplier.get()
    // .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
    // .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getSpeed(), 0)));
    // }

    // // @Log(groups = "components")
    // public Pose3d getClawComponentPose() {
    // return getArmComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
    // }

    // @Log
    @Override
    public double getSpeed() {
        return m_algaeIntakeMotor.getVelocity().getValueAsDouble();
    }

    // @Log
    // public double getVelocity() {
    // // if (RobotBase.isReal())
    // return m_algaeIntakeMotor.getVelocity().getValueAsDouble();
    // // else
    // // return simVelocity;
    // }

    @Override
    public void resetSpeed() {
        // m_algaeIntakeMotor.set(AlgaeIntakeSpeed.STOPPED.value);
        m_algaeIntakeMotor.stopMotor();

        initialized = true;
    }

    // @Override
    // public void setVoltage(double voltage) {
    // voltage = MathUtil.clamp(voltage, -12, 12);
    // // voltage = Utils.applySoftStops(voltage, getPosition(), ArmConstants.MIN_ANGLE_RADIANS,
    // // ArmConstants.MAX_ANGLE_RADIANS);

    // // if (voltage < 0
    // // && getPosition() < 0
    // // && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT) {
    // // voltage = 0;
    // // }

    // // if (!GlobalStates.INITIALIZED.enabled()) {
    // // voltage = 0.0;
    // // }

    // m_armMotor.setVoltage(voltage);
    // }

    public void moveToSpeed(double goalVelocity) {
        // Safety check: prevent movement until robot is initialized
        if (!RobotState.getInstance().isInitialized()) {
            goalVelocity = 0.0;
        }
        // VelocityVoltage velocityOut = new VelocityVoltage(0);
        // velocityOut.Slot = 0;
        // m_algaeIntakeMotor.setControl(velocityOut.withVelocity(goalVelocity));
        m_algaeIntakeMotor.set(goalVelocity);
    }

    // @Override
    // public Command moveToCurrentGoalCommand() {
    // return run(() -> {
    // feedbackVoltage = pidController.calculate(getPosition());
    // // not the setpoint position, as smart people found that using the current
    // // position for kG works best
    // feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity);
    // setVoltage(feedbackVoltage + feedforwardVoltage);
    // }).withName("arm.moveToCurrentGoal");
    // }
    private Command moveToSpeedCommand(double goalVelocity) {
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized()) {
                VelocityVoltage velocityOut = new VelocityVoltage(0);
                velocityOut.Slot = 0;
                m_algaeIntakeMotor.setControl(velocityOut.withVelocity(goalVelocity));
            } else {
                m_algaeIntakeMotor.stopMotor();
            }
        })
                // .until(() -> Math.abs(getSpeed() - goalVelocity) < .5)
                .withName("algaeIntake.moveToSpeed");

        // return run(() -> {
        // m_algaeIntakeMotor.setControl(m_request.withVelocity(goalVelocity));
        // algaeIntakePub.set(m_positionTracker.getAlgaeIntakeSpeed());
        // })
        // .until(() -> Math.abs(getSpeed() - goalVelocity) < .5)
        // .withName("algaeIntake.moveToSpeed");
    }

    @Override
    public Command moveToSetSpeedCommand(Supplier<AlgaeIntakeSpeed> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get().value))
                .withTimeout(3)
                .withName("algaeIntake.moveToSetSpeed");
    }

    @Override
    public Command moveToArbitrarySpeedCommand(Supplier<Double> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get())).withName("algaeIntake.moveToArbitrarySpeed");
    }

    @Override
    public Command moveSpeedDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(getSpeed() + delta.get())).withName("algaeIntake.moveSpeedDelta");
    }

    // @Override
    // public Command holdCurrentPositionCommand() {
    // return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
    // .withName("algaeIntake.holdCurrentPosition");
    // }

    @Override
    public Command resetSpeedCommand() {
        return runOnce(this::resetSpeed).withName("algaeIntake.resetSpeed");
    }

    // @Override
    // public Command setOverridenSpeedCommand(Supplier<Double> speed) {
    // return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
    // .withName("algaeIntake.setOverriddenSpeed");
    // }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> m_algaeIntakeMotor.stopMotor())
                .andThen(() -> m_algaeIntakeMotor.setNeutralMode(NeutralModeValue.Coast))
                .finallyDo((d) -> {
                    // motor.setIdleMode(IdleMode.kBrake);
                    // pidController.reset(getPosition());
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("algaeIntake.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("algaeIntake.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("algaeIntake.sysIdDynamic");
    }

    // public Command resetControllersCommand() {
    // return Commands.runOnce(() -> pidController.reset(getPosition()))
    // .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    // }

    // public boolean atGoal() {
    // return pidController.atGoal();
    // }

    /**
     * Cleans up resources when the robot is disabled.
     * Stops the motor and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_algaeIntakeMotor != null) {
            m_algaeIntakeMotor.stopMotor();
        }
        if (algaeIntakePub != null) {
            algaeIntakePub.close();
        }
    }
}
