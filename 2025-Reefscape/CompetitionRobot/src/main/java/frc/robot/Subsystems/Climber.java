package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.IDs.CLIMBER_ID;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionTracker;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Subsystems.Bases.BaseSingleJointedArm;

public class Climber extends SubsystemBase implements BaseSingleJointedArm<ClimberPosition> {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Mechanisms");
    private final DoublePublisher climberPub = table.getDoubleTopic("Climber Position").publish();

    private TalonFX m_climber;

    private final PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;
    // private final Supplier<Pose3d> carriagePoseSupplier;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean initialized;

    public Climber(PositionTracker positionTracker) {
            /* MechanismLigament2d ligament */
            // Supplier<Pose3d> carriagePoseSupplier) {
        m_positionTracker = positionTracker;
        positionTracker.setClimberPositionSupplier(this::getPosition);

        // this.carriagePoseSupplier = carriagePoseSupplier;

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
        motionMagicConfigs.MotionMagicCruiseVelocity = 50; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_climber = new TalonFX(CLIMBER_ID);
        m_climber.setNeutralMode(NeutralModeValue.Brake);

        // talonFxConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonFxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_climber.getConfigurator().apply(talonFxConfigs);

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
    public Pose3d getClimberComponentPose() {
        return null;
        // return carriagePoseSupplier.get()
        //         .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
        //         .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
    }

    // @Log(groups = "components")
    // public Pose3d getClawComponentPose() {
    //     return getClimberComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
    // }

    // @Log
    @Override
    public double getPosition() {
        return m_climber.getPosition().getValueAsDouble();
    }

    // @Log
    public double getVelocity() {
        // if (RobotBase.isReal())
        return m_climber.getVelocity().getValueAsDouble();
        // else
        // return simVelocity;
    }

    @Override
    public void resetPosition() {
        m_climber.setPosition(ClimberPosition.DOWN.value);

        initialized = true;
    }

    // @Override
    // public void setVoltage(double voltage) {
    // voltage = MathUtil.clamp(voltage, -12, 12);
    // voltage = Utils.applySoftStops(voltage, getPosition(), ClimberConstants.MIN_ANGLE_RADIANS,
    // ClimberConstants.MAX_ANGLE_RADIANS);

    // if (voltage < 0
    // && getPosition() < 0
    // && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT) {
    // voltage = 0;
    // }

    // if (!GlobalStates.INITIALIZED.enabled()) {
    // voltage = 0.0;
    // }

    // m_climber.setVoltage(voltage);
    // }

    // @Override
    // public Command moveToCurrentGoalCommand() {
    // return run(() -> {
    // feedbackVoltage = pidController.calculate(getPosition());
    // // not the setpoint position, as smart people found that using the current
    // // position for kG works best
    // feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity);
    // setVoltage(feedbackVoltage + feedforwardVoltage);
    // }).withName("climber.moveToCurrentGoal");
    // }
    private Command moveToPositionCommand(double goalPosition) {
        return run(() -> {
            m_climber.setControl(m_request.withPosition(goalPosition));
            climberPub.set(m_positionTracker.getClimberPosition());
        })
        .until(() -> Math.abs(getPosition() - goalPosition) < .5 || Math.abs(getPosition()) > Math.abs(goalPosition)) //abs(goal - position) < error or abs(position) > abs(goal) (so it can't move backwards) 
        .withName("climber.moveToPosition");
    }

    @Override
    public Command moveToSetPositionCommand(Supplier<ClimberPosition> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get().value))
                .withTimeout(2.5)
                .withName("climber.moveToSetPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get())).withName("climber.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                moveToPositionCommand(getPosition() + delta.get())).withName("climber.movePositionDelta");
    }

    // @Override
    // public Command holdCurrentPositionCommand() {
    // return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
    // .withName("climber.holdCurrentPosition");
    // }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("climber.resetPosition");
    }

    // @Override
    // public Command setOverridenSpeedCommand(Supplier<Double> speed) {
    // return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
    // .withName("climber.setOverriddenSpeed");
    // }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> m_climber.stopMotor())
                .andThen(() -> m_climber.setNeutralMode(NeutralModeValue.Coast))
                .finallyDo((d) -> {
                    // motor.setIdleMode(IdleMode.kBrake);
                    // pidController.reset(getPosition());
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("climber.coastMotorsCommand");
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
