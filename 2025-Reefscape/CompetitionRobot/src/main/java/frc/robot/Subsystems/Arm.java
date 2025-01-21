package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalStates;
import frc.robot.PositionTracker;
import frc.robot.Utils;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Bases.BaseSingleJointedArm;

import static frc.robot.Constants.Constants.IDs.ARM_ID;

import static frc.robot.Constants.ArmConstants.kP;
import static frc.robot.Constants.ArmConstants.kI;
import static frc.robot.Constants.ArmConstants.kD;
import static frc.robot.Constants.ArmConstants.kS;
import static frc.robot.Constants.ArmConstants.kG;
import static frc.robot.Constants.ArmConstants.kV;
import static frc.robot.Constants.ArmConstants.kA;
import static frc.robot.Constants.ArmConstants.MOVEMENT_CONSTRAINTS;

// @LoggedObject
public class Arm extends SubsystemBase implements BaseSingleJointedArm<ArmPosition> {
    // @Log
    // private final CANSparkMax motor;

    // @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    // @Log(groups = "control")
    private final ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    // private final SingleJointedArmSim armSim = new SingleJointedArmSim(
    //         MOTOR_GEARBOX_REPR,
    //         GEARING,
    //         MOI,
    //         COM_DISTANCE_METERS,
    //         MIN_ANGLE_RADIANS,
    //         MAX_ANGLE_RADIANS,
    //         true,
    //         ArmPosition.TOP.value);

    // @Log(groups = "control")
    private double feedbackVoltage = 0;
    // @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    // private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    // private final MutableMeasure<Angle> sysidPositionMeasure = MutableMeasure.mutable(Radians.of(0));
    // private final MutableMeasure<Velocity<Angle>> sysidVelocityMeasure = MutableMeasure
    //         .mutable(RadiansPerSecond.of(0));

    // private final SysIdRoutine sysIdRoutine;

    private final PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;
    private final Supplier<Pose3d> carriagePoseSupplier;

    private TalonFX m_arm;

    private double LOAD_ENCODER_VAL = 2; //Actual Stowed Value: 0
    // private double EXTEND_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double SCORE_ENCODER_VAL = 300; //Actual Deploy Value: 320

    // @Log
    private boolean initialized;

    public Arm(PositionTracker positionTracker/*, MechanismLigament2d ligament*/, Supplier<Pose3d> carriagePoseSupplier) {
        // m_arm = TalonFXConfigurator.MOTOR_ID, MotorType.kBrushless, MOTOR_INVERTED,
        //         (s) -> s.setIdleMode(IdleMode.kBrake),
        //         (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
        //         (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
        //         (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        // sysIdRoutine = new SysIdRoutine(
        //         new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(3), null, null),
        //         new SysIdRoutine.Mechanism(
        //                 volts -> setVoltage(volts.magnitude()),
        //                 null,
        //                 // log -> {
        //                 //     log.motor("arm")
        //                 //             .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
        //                 //             .angularPosition(sysidPositionMeasure.mut_replace(getPosition(), Radians))
        //                 //             .angularVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), RadiansPerSecond));
        //                 // },
        //                 this
        //         )
        //     );

        m_positionTracker = positionTracker;
        // this.ligament = ligament;
        this.carriagePoseSupplier = carriagePoseSupplier;

        m_positionTracker.setArmAngleSupplier(this::getPosition);

        // setDefaultCommand(moveToCurrentGoalCommand());

        m_arm = new TalonFX(ARM_ID);
        // m_elevator1.setInverted(false);
        m_arm.setNeutralMode(NeutralModeValue.Brake);
        //m_elevatorPivot.setPosition(0);
        var configs1 = new TalonFXConfiguration();
        configs1.Slot0.kG = 0;
        configs1.Slot0.kS = 0;
        configs1.Slot0.kP = 0;
        configs1.Slot0.kI = 0;
        configs1.Slot0.kD = 0;
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        configs1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_arm.getConfigurator().apply(configs1);
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

    // return new Pose3d(0.168, 0, 0.247, new Rotation3d());
    // -0.083

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
        return m_arm.getPosition().getValueAsDouble();
    }

    // @Log
    public double getVelocity() {
        if (RobotBase.isReal())
            return m_arm.getVelocity().getValueAsDouble();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        m_arm.setPosition(LOAD_ENCODER_VAL);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), ArmConstants.MIN_ANGLE_RADIANS, ArmConstants.MAX_ANGLE_RADIANS);

        if (voltage < 0
                && getPosition() < 0
                && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT) {
            voltage = 0;
        }

        if (!GlobalStates.INITIALIZED.enabled()) {
            voltage = 0.0;
        }

        m_arm.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            // not the setpoint position, as smart people found that using the current
            // position for kG works best
            feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("arm.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ArmPosition> goalPositionSupplier) {
        // return null;
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                .withTimeout(3)
                .withName("arm.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("arm.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("arm.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("arm.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("arm.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("arm.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        // return null;
        return runOnce(m_arm::stopMotor)
                // .andThen(() -> m_arm.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    // m_arm.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("arm.coastMotorsCommand");
    }

    // public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    // }

    // public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    // }

    // public Command resetControllersCommand() {
    //     return Commands.runOnce(() -> pidController.reset(getPosition()))
    //             .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    // }

    public boolean atGoal() {
        return pidController.atGoal();
    }
}
