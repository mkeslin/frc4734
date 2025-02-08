package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.IDs.ELEVATOR_LEFT_ID;
import static frc.robot.Constants.Constants.IDs.ELEVATOR_RIGHT_ID;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionTracker;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Subsystems.Bases.BaseLinearMechanism;

public class Elevator extends SubsystemBase implements BaseLinearMechanism<ElevatorPosition> {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Mechanisms");
    private final DoublePublisher elevatorPub = table.getDoubleTopic("Elevator Position").publish();

    private TalonFX m_elevatorLeftLeaderMotor;
    private TalonFX m_elevatorRightFollowerMotor;

    // private double simVelocity = 0.0;

    private final PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean initialized;

    // private final ElevatorSim elevatorSim = new ElevatorSim(
    // MOTOR_GEARBOX_REPR,
    // GEARING,
    // MASS_KG,
    // DRUM_RADIUS_METERS,
    // MIN_HEIGHT_METERS,
    // MAX_HEIGHT_METERS,
    // true,
    // ElevatorPosition.BOTTOM.value);

    public Elevator(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
        positionTracker.setElevatorPositionSupplier(this::getPosition);

        var talonFxConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFxConfigs.Slot0;
        slot0Configs.kG = 0.0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 20.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 80.0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFxConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFxConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_elevatorLeftLeaderMotor = new TalonFX(ELEVATOR_LEFT_ID);
        m_elevatorLeftLeaderMotor.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorLeftLeaderMotor.getConfigurator().apply(talonFxConfigs);

        m_elevatorRightFollowerMotor = new TalonFX(ELEVATOR_RIGHT_ID);
        m_elevatorLeftLeaderMotor.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorRightFollowerMotor.setControl(new Follower(m_elevatorLeftLeaderMotor.getDeviceID(), false));

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
    // return new Pose3d(0.14, 0, 0.13, new Rotation3d());
    // }

    // @Log(groups = "components")
    // public Pose3d getStageComponentPose() {
    // Transform3d transform = new Transform3d();
    // if (getPosition() > 0.706) {
    // transform = new Transform3d(0, 0, getPosition() - 0.706, new Rotation3d());
    // }
    // return new Pose3d(0.14, 0, 0.169, new Rotation3d()).plus(transform);
    // }

    // @Log(groups = "components")
    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.14, 0, 0.247 + getPosition(), new Rotation3d());
    }

    @Override
    public double getPosition() {
        return m_elevatorLeftLeaderMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        // if (RobotBase.isReal())
        return m_elevatorLeftLeaderMotor.getVelocity().getValueAsDouble();
        // else
        // return simVelocity;
    }

    @Override
    public void resetPosition() {
        m_elevatorLeftLeaderMotor.setPosition(ElevatorPosition.BOTTOM.value);
        m_elevatorRightFollowerMotor.setPosition(ElevatorPosition.BOTTOM.value);

        initialized = true;
    }

    // @Override
    // public void setVoltage(double voltage) {
    //     voltage = MathUtil.clamp(voltage, -12, 12);
    //     // voltage = Utils.applySoftStops(voltage, getPosition(), ElevatorConstants.MIN_HEIGHT_METERS,
    //     //         ElevatorConstants.MAX_HEIGHT_METERS);

    //     // if (voltage < 0
    //     //         && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT
    //     //         && m_positionTracker.getArmAngle() < 0) {
    //     //     voltage = 0;
    //     // }

    //     // if (!GlobalStates.INITIALIZED.enabled()) {
    //     // voltage = 0.0;
    //     // }

    //     // voltage = .4;

    //     m_elevatorLeftLeaderMotor.setVoltage(voltage);
    //     m_elevatorRightFollowerMotor.setVoltage(-voltage);
    // }

    // @Override
    private Command moveToPositionCommand(double goalPosition) {
        return run(() -> {
            m_elevatorLeftLeaderMotor.setControl(m_request.withPosition(goalPosition));
            elevatorPub.set(m_positionTracker.getElevatorPosition());
        })
        .until(() -> Math.abs(getPosition() - goalPosition) < .5) //abs(goal - position) < error 
        .withName("elevator.moveToPosition");
    }

    @Override
    public Command moveToSetPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get().value))
                .withTimeout(3)
                .withName("elevator.moveToSetPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get()))
                .withName("elevator.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                moveToPositionCommand(getPosition() + delta.get()))
                .withName("elevator.movePositionDelta");
    }

    // @Override
    // public Command holdCurrentPositionCommand() {
    // return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
    // .withName("elevator.holdCurrentPosition");
    // }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

    // @Override
    // public Command setOverridenSpeedCommand(Supplier<Double> speed) {
    // return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
    // .withName("elevator.setOverriddenSpeed");
    // }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            m_elevatorLeftLeaderMotor.stopMotor();
            m_elevatorRightFollowerMotor.stopMotor();
        })
                .andThen(() -> {
                    m_elevatorLeftLeaderMotor.setNeutralMode(NeutralModeValue.Coast);
                    m_elevatorRightFollowerMotor.setNeutralMode(NeutralModeValue.Coast);
                })
                .finallyDo((d) -> {
                    // motor.setIdleMode(IdleMode.kBrake);
                    // pidController.reset(getPosition());
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsCommand");
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
    // return pidController.atGoal();
    // }
}
