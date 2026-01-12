package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIds.ELEVATOR_LEFT;
import static frc.robot.Constants.CANIds.ELEVATOR_RIGHT;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Subsystems.Bases.BaseLinearMechanism;

public class Elevator extends SubsystemBase implements BaseLinearMechanism<ElevatorPosition> {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Mechanisms");
    private final DoublePublisher elevatorPub = table.getDoubleTopic("Elevator Position").publish();

    private TalonFX m_elevatorLeftLeaderMotor;
    private TalonFX m_elevatorRightFollowerMotor;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutine;

    private PositionTracker m_positionTracker;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean initialized;

    public Elevator() {

        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(15).div(Seconds.of(1)),
                        Volts.of(7),
                        null,
                        (state) -> SignalLogger.writeString("Elevator State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            m_elevatorLeftLeaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
                            m_elevatorRightFollowerMotor.setControl(m_voltReq.withOutput(volts.in(Volts)));
                        },
                        null,
                        this,
                        "Arm"));

        var talonFxConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFxConfigs.Slot0;
        slot0Configs.kG = 0.0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 20.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 82.0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFxConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFxConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_elevatorLeftLeaderMotor = new TalonFX(ELEVATOR_LEFT);
        m_elevatorLeftLeaderMotor.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorLeftLeaderMotor.getConfigurator().apply(talonFxConfigs);

        m_elevatorRightFollowerMotor = new TalonFX(ELEVATOR_RIGHT);
        m_elevatorRightFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorRightFollowerMotor.setControl(new Follower(m_elevatorLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        resetPosition();
    }

    @Override
    public void simulationPeriodic() {
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

    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.14, 0, 0.247 + getPosition(), new Rotation3d());
    }

    @Override
    public double getPosition() {
        return m_elevatorLeftLeaderMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_elevatorLeftLeaderMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        m_elevatorLeftLeaderMotor.setPosition(ElevatorPosition.BOTTOM.value);
        m_elevatorRightFollowerMotor.setPosition(ElevatorPosition.BOTTOM.value);

        initialized = true;
    }

    private Command moveToPositionCommand(double goalPosition) {
        return run(() -> {
            m_elevatorLeftLeaderMotor.setControl(m_request.withPosition(goalPosition));
            if (m_positionTracker != null) {
                elevatorPub.set(m_positionTracker.getElevatorPosition());
            }
        })
                .until(() -> Math.abs(getPosition() - goalPosition) < .5)
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

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

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
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    }
}
