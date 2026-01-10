package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Constants.IDs.SIDE_TO_SIDE_ID;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.Constants.SideToSideConstants.SideToSidePosition;
import frc.robot.Subsystems.Bases.BaseLinearMechanism;

public class SideToSide extends SubsystemBase implements BaseLinearMechanism<SideToSidePosition> {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Mechanisms");
    private final DoublePublisher sideToSidePub = table.getDoubleTopic("Side To Side Position").publish();

    private TalonFX m_sideToSideMotor;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    // private double simVelocity = 0.0;

    private final SysIdRoutine m_sysIdRoutine;

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

    public SideToSide(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
        positionTracker.setSideToSidePositionSupplier(this::getPosition);

        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).div(Seconds.of(1)),
                        Volts.of(3),
                        null,
                        (state) -> SignalLogger.writeString("SideToSide State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_sideToSideMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null,
                        this,
                        "Arm"));

        var talonFxConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFxConfigs.Slot0;
        slot0Configs.kG = 0.0;
        slot0Configs.kS = 0.25;
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01;
        slot0Configs.kP = 4.8;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        // slot0Configs.kG = 0.0;
        // slot0Configs.kS = 0.078998;
        // slot0Configs.kV = 0.1078;
        // slot0Configs.kA = 0.00105;
        // slot0Configs.kP = 58.889;
        // slot0Configs.kI = 0.0;
        // slot0Configs.kD = 0.32033;

        // set Motion Magic settings
        var motionMagicConfigs = talonFxConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 55; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_sideToSideMotor = new TalonFX(SIDE_TO_SIDE_ID);
        m_sideToSideMotor.setNeutralMode(NeutralModeValue.Brake);

        // talonFxConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonFxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_sideToSideMotor.getConfigurator().apply(talonFxConfigs);

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
        m_sideToSideMotor.setPosition(SideToSidePosition.CENTER.value);

        initialized = true;
    }

    private Command moveToPositionCommand(double goalPosition) {
        return run(() -> {
            m_sideToSideMotor.setControl(m_request.withPosition(goalPosition));
            sideToSidePub.set(m_positionTracker.getSideToSidePosition());
        })
                .until(() -> Math.abs(getPosition() - goalPosition) < .5) // ||                         // abs(goal - position) < error
                        // m_positionTracker.getElevatorPosition() < ElevatorPosition.INTAKE_PREP)     // don't move if too low
                .withName("sideToSide.moveToPosition");
    }

    @Override
    public Command moveToSetPositionCommand(Supplier<SideToSidePosition> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get().value))
                .withTimeout(3)
                .withName("sideToSide.moveToSetPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get())).withName("sideToSide.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                moveToPositionCommand(getPosition() + delta.get())).withName("sideToSide.movePositionDelta");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("sideToSide.resetPosition");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> m_sideToSideMotor.stopMotor())
                .andThen(() -> m_sideToSideMotor.setNeutralMode(NeutralModeValue.Coast))
                .finallyDo((d) -> {
                    // motor.setIdleMode(IdleMode.kBrake);
                    // pidController.reset(getPosition());
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("sideToSide.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("sideToSide.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("sideToSide.sysIdDynamic");
    }

    // public Command resetControllersCommand() {
    // return Commands.runOnce(() -> pidController.reset(getPosition()))
    // .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    // }

    // public boolean atGoal() {
    // // return pidController.atGoal();
    // return false;
    // }
}
