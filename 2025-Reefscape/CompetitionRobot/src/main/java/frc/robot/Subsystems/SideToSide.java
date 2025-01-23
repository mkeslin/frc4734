package frc.robot.Subsystems;

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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalStates;
import frc.robot.PositionTracker;
import frc.robot.Utils;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.SideToSideConstants;
import frc.robot.Constants.SideToSideConstants.SideToSidePosition;
import frc.robot.Subsystems.Bases.BaseLinearMechanism;

import static frc.robot.Constants.Constants.IDs.SIDE_TO_SIDE_ID;

import static frc.robot.Constants.SideToSideConstants.kP;
import static frc.robot.Constants.SideToSideConstants.kI;
import static frc.robot.Constants.SideToSideConstants.kD;
import static frc.robot.Constants.SideToSideConstants.kS;
import static frc.robot.Constants.SideToSideConstants.kG;
import static frc.robot.Constants.SideToSideConstants.kV;
import static frc.robot.Constants.SideToSideConstants.kA;
import static frc.robot.Constants.SideToSideConstants.MOVEMENT_CONSTRAINTS;

import java.util.function.Supplier;

public class SideToSide extends SubsystemBase implements BaseLinearMechanism<SideToSidePosition> {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Mechanisms");
    private final DoublePublisher sideToSidePub = table.getDoubleTopic("Side To Side").publish();

    private TalonFX m_sideToSideMotor;

    private double simVelocity = 0.0;

    // @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    // @Log(groups = "control")
    private final ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    // private double RETRACT_ENCODER_VAL = 2; //Actual Stowed Value: 0
    // private double EXTEND_L1_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double EXTEND_L2_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double EXTEND_L3_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double EXTEND_L4_ENCODER_VAL = 300; //Actual Deploy Value: 320

    private final PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;

    // @Log(groups = "control")
    private double feedbackVoltage = 0;
    // @Log(groups = "control")
    private double feedforwardVoltage = 0;

    // @Log
    private boolean initialized;

    // private final ElevatorSim elevatorSim = new ElevatorSim(
    //     MOTOR_GEARBOX_REPR,
    //     GEARING,
    //     MASS_KG,
    //     DRUM_RADIUS_METERS,
    //     MIN_HEIGHT_METERS,
    //     MAX_HEIGHT_METERS,
    //     true,
    //     ElevatorPosition.BOTTOM.value);

    public SideToSide(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
        positionTracker.setSideToSidePositionSupplier(this::getPosition);

        // // set slot 0 gains
        // var slot0Configs = talonFXConfigs.Slot0;
        // // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        // // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // // slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        // slot0Configs.kI = 0.001; // no output for integrated error
        // slot0Configs.kD = 0.69; // A velocity error of 1 rps results in 0.1 V output

        // // set Motion Magic settings
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 739; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 369; // Target acceleration of 160 rps/s (0.5 seconds)
        // // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // motor1.getConfigurator().apply(talonFXConfigs);
        // motor2.getConfigurator().apply(talonFXConfigs);

        // motor1.config_kD(0, 0.69, 0);
        // motor2.config_kD(0, 0.69, 0);
        // motor1.config_kI(0, 0.001, 0);
        // motor2.config_kI(0, 0.001, 0);
        // motor1.config_kF(0, 0.069, 0);
        // motor2.config_kF(0, 0.069, 0);
        // motor1.configMotionCruiseVelocity(739, 0);
        // motor2.configMotionCruiseVelocity(739, 0);
        // motor1.configMotionAcceleration(369, 0);
        // motor2.configMotionAcceleration(369, 0);

        m_sideToSideMotor = new TalonFX(SIDE_TO_SIDE_ID);
        // m_elevator1.setInverted(false);
        m_sideToSideMotor.setNeutralMode(NeutralModeValue.Brake);
        //m_elevatorPivot.setPosition(0);
        var configs1 = new TalonFXConfiguration();
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        configs1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_sideToSideMotor.getConfigurator().apply(configs1);

        // zeroValue = zero;
        // halfValue = half;
        // fullValue = full;
        // name = n;
        // m1EncoderVal = 0;
        // m2EncoderVal = 0;
        // elevatorMovingIn = false;
        // elevatorMovingOut = false;

        resetPosition();
    }

    @Override
    public void simulationPeriodic() {
        // elevatorSim.setInput(motor.getAppliedOutput());
        // elevatorSim.update(0.020);
        // motor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        // simVelocity = elevatorSim.getVelocityMetersPerSecond();

        // ligament.setLength(getPosition());
    }

    public boolean getInitialized() {
        return initialized;
    }

    // @Log(groups = "components")
    // public Pose3d getFrameComponentPose() {
    //     return new Pose3d(0.14, 0, 0.13, new Rotation3d());
    // }

    // @Log(groups = "components")
    // public Pose3d getStageComponentPose() {
    //     Transform3d transform = new Transform3d();
    //     if (getPosition() > 0.706) {
    //         transform = new Transform3d(0, 0, getPosition() - 0.706, new Rotation3d());
    //     }
    //     return new Pose3d(0.14, 0, 0.169, new Rotation3d()).plus(transform);
    // }

    // @Log(groups = "components")
    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.14, 0, 0.247 + getPosition(), new Rotation3d());
    }

    @Override
    public double getPosition() {
        return m_sideToSideMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        // if (RobotBase.isReal())
            return m_sideToSideMotor.getVelocity().getValueAsDouble();
        // else
            // return simVelocity;
    }

    @Override
    public void resetPosition() {
        m_sideToSideMotor.setPosition(SideToSidePosition.LEFT.value);

        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), SideToSideConstants.MIN_HEIGHT_METERS, SideToSideConstants.MAX_HEIGHT_METERS);

        if (voltage < 0
                && m_positionTracker.getSideToSidePosition() < SideToSideConstants.MOTION_LIMIT
                && m_positionTracker.getArmAngle() < 0) {
            voltage = 0;
        }

        sideToSidePub.set(m_positionTracker.getSideToSidePosition());

        // if (!GlobalStates.INITIALIZED.enabled()) {
        //     voltage = 0.0;
        // }

        // voltage = .4;

        m_sideToSideMotor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("sideToSide.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<SideToSidePosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                // .withTimeout(3)
                .withName("sideToSide.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("sideToSide.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("sideToSide.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("sideToSide.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("sideToSide.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("sideToSide.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(this::stopMotors)
                // .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    // motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("sideToSide.coastMotorsCommand");
    }

    private void stopMotors() {
        m_sideToSideMotor.stopMotor();
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
