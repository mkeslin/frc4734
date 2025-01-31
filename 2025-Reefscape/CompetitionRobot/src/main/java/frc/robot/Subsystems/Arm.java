package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.IDs.ARM_ID;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionTracker;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Subsystems.Bases.BaseSingleJointedArm;

// @LoggedObject
public class Arm extends SubsystemBase implements BaseSingleJointedArm<ArmPosition> {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Mechanisms");
    private final DoublePublisher armPub = table.getDoubleTopic("Arm Angle").publish();

    private TalonFX m_armMotor;

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

    // private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    // private final MutableMeasure<Angle> sysidPositionMeasure = MutableMeasure.mutable(Radians.of(0));
    // private final MutableMeasure<Velocity<Angle>> sysidVelocityMeasure = MutableMeasure
    // .mutable(RadiansPerSecond.of(0));

    // private final SysIdRoutine sysIdRoutine;

    private final PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;
    private final Supplier<Pose3d> carriagePoseSupplier;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean initialized;

    public Arm(
            PositionTracker positionTracker,
            /* MechanismLigament2d ligament */
            Supplier<Pose3d> carriagePoseSupplier) {
        m_positionTracker = positionTracker;
        positionTracker.setArmAngleSupplier(this::getPosition);

        this.carriagePoseSupplier = carriagePoseSupplier;

        // m_arm = TalonFXConfigurator.MOTOR_ID, MotorType.kBrushless, MOTOR_INVERTED,
        // (s) -> s.setIdleMode(IdleMode.kBrake),
        // (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
        // (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
        // (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        // sysIdRoutine = new SysIdRoutine(
        // new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(3), null, null),
        // new SysIdRoutine.Mechanism(
        // volts -> setVoltage(volts.magnitude()),
        // null,
        // // log -> {
        // // log.motor("arm")
        // // .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
        // // .angularPosition(sysidPositionMeasure.mut_replace(getPosition(), Radians))
        // // .angularVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), RadiansPerSecond));
        // // },
        // this
        // )
        // );

        // setDefaultCommand(moveToCurrentGoalCommand());

        var talonFxConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFxConfigs.Slot0;
        slot0Configs.kG = -0.2;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFxConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_armMotor = new TalonFX(ARM_ID);
        m_armMotor.setNeutralMode(NeutralModeValue.Brake);

        // talonFxConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonFxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_armMotor.getConfigurator().apply(talonFxConfigs);

        resetPosition();
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

    // @Log(groups = "components")
    public Pose3d getArmComponentPose() {
        return carriagePoseSupplier.get()
                .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
                .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
    }

    // @Log(groups = "components")
    public Pose3d getClawComponentPose() {
        return getArmComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
    }

    // @Log
    @Override
    public double getPosition() {
        return m_armMotor.getPosition().getValueAsDouble();
    }

    // @Log
    public double getVelocity() {
        // if (RobotBase.isReal())
        return m_armMotor.getVelocity().getValueAsDouble();
        // else
        // return simVelocity;
    }

    @Override
    public void resetPosition() {
        m_armMotor.setPosition(ArmPosition.BOTTOM.value);

        initialized = true;
    }

    // @Override
    // public void setVoltage(double voltage) {
    // voltage = MathUtil.clamp(voltage, -12, 12);
    // voltage = Utils.applySoftStops(voltage, getPosition(), ArmConstants.MIN_ANGLE_RADIANS,
    // ArmConstants.MAX_ANGLE_RADIANS);

    // if (voltage < 0
    // && getPosition() < 0
    // && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT) {
    // voltage = 0;
    // }

    // if (!GlobalStates.INITIALIZED.enabled()) {
    // voltage = 0.0;
    // }

    // m_arm.setVoltage(voltage);
    // }

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
    private Command moveToPositionCommand(double goalPosition) {
        return run(() -> {
            m_armMotor.setControl(m_request.withPosition(goalPosition));
            armPub.set(m_positionTracker.getArmAngle());
        }).withName("arm.moveToPosition");
    }

    @Override
    public Command moveToSetPositionCommand(Supplier<ArmPosition> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get().value)).withName("arm.moveToSetPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get())).withName("arm.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                moveToPositionCommand(getPosition() + delta.get())).withName("arm.movePositionDelta");
    }

    // @Override
    // public Command holdCurrentPositionCommand() {
    //     return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
    //             .withName("arm.holdCurrentPosition");
    // }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("arm.resetPosition");
    }

    // @Override
    // public Command setOverridenSpeedCommand(Supplier<Double> speed) {
    //     return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
    //             .withName("arm.setOverriddenSpeed");
    // }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> m_armMotor.stopMotor())
                .andThen(() -> m_armMotor.setNeutralMode(NeutralModeValue.Coast))
                .finallyDo((d) -> {
                    // motor.setIdleMode(IdleMode.kBrake);
                    // pidController.reset(getPosition());
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("sideToSide.coastMotorsCommand");
    }

    // public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    // }

    // public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    // }

    // public Command resetControllersCommand() {
    // return Commands.runOnce(() -> pidController.reset(getPosition()))
    // .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    // }

    // public boolean atGoal() {
    //     return pidController.atGoal();
    // }
}
