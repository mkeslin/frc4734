package frc.robot.Subsystems;

import static frc.robot.Constants.CANIds.CLIMBER;
import static frc.robot.Constants.CANIds.CLIMBER_2;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.Telemetry;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Subsystems.Bases.BaseSingleJointedArm;

/**
 * Climber subsystem that controls the robot's climbing mechanism.
 * Uses two TalonFX motors (leader and follower) with MotionMagic control for smooth position-based movement.
 * The climber can move to predefined positions (via ClimberPosition enum) or arbitrary positions.
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Uses MotionMagic for smooth, controlled motion</li>
 *   <li>Publishes climber position to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseSingleJointedArm
 * @see ClimberPosition
 * @see RobotState
 */
public class Climber extends SubsystemBase implements BaseSingleJointedArm<ClimberPosition> {
    private final DoublePublisher climberPub = Telemetry.createMechanismsPublisher("Climber Position");

    private TalonFX m_climber;
    private TalonFX m_climber2;

    private PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;
    // private final Supplier<Pose3d> carriagePoseSupplier;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean initialized;

    public Climber() {

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
        motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_climber = new TalonFX(CLIMBER);
        m_climber.setNeutralMode(NeutralModeValue.Brake);

        // talonFxConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonFxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_climber.getConfigurator().apply(talonFxConfigs);

        m_climber2 = new TalonFX(CLIMBER_2);
        m_climber2.setNeutralMode(NeutralModeValue.Brake);
        m_climber2.setControl(new Follower(m_climber.getDeviceID(), MotorAlignmentValue.Aligned));

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

    /**
     * Updates the PositionTracker reference. Used during initialization to ensure
     * all subsystems share the same PositionTracker instance with real suppliers.
     */
    public void setPositionTracker(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
    }

    // @Log(groups = "components")
    public Pose3d getClimberComponentPose() {
        return null;
        // return carriagePoseSupplier.get()
        // .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
        // .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
    }

    // @Log(groups = "components")
    // public Pose3d getClawComponentPose() {
    // return getClimberComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
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
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // voltage = Utils.applySoftStops(voltage, getPosition(), ClimberConstants.MIN_ANGLE_RADIANS,
        // ClimberConstants.MAX_ANGLE_RADIANS);

        // if (voltage < 0
        // && getPosition() < 0
        // && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT) {
        // voltage = 0;
        // }

        // Safety check: prevent movement until robot is initialized
        if (!RobotState.getInstance().isInitialized()) {
            voltage = 0.0;
        }

        m_climber.setVoltage(voltage);
        // Follower motor automatically follows leader
    }

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
        // var currentPosition = getPosition();
        // if(Math.abs(getPosition()) > Math.abs(goalPosition)) {
        //     return Commands.none();
        // }
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized()) {
                m_climber.setControl(m_request.withPosition(goalPosition));
                // Follower motor automatically follows leader
            } else {
                m_climber.stopMotor();
                m_climber2.stopMotor();
            }
            if (m_positionTracker != null) {
                climberPub.set(m_positionTracker.getClimberPosition());
            }
        })
                .until(() -> Math.abs(getPosition() - goalPosition) < .5) // abs(goal - position) < error or
                                                                             // abs(position) > abs(goal) (so it can't
                                                                             // move backwards)
                .withName("climber.moveToPosition");
    }

    @Override
    public Command moveToSetPositionCommand(Supplier<ClimberPosition> goalPositionSupplier) {
        Objects.requireNonNull(goalPositionSupplier, "goalPositionSupplier cannot be null");
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get().value))
                .withTimeout(15)
                .withName("climber.moveToSetPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        Objects.requireNonNull(goalPositionSupplier, "goalPositionSupplier cannot be null");
        return Commands.sequence(
                moveToPositionCommand(goalPositionSupplier.get())).withName("climber.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
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
        return runOnce(() -> {
            m_climber.stopMotor();
            m_climber2.stopMotor();
        })
                .andThen(() -> {
                    m_climber.setNeutralMode(NeutralModeValue.Coast);
                    m_climber2.setNeutralMode(NeutralModeValue.Coast);
                })
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

    /**
     * Cleans up resources when the robot is disabled.
     * Stops all motors and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_climber != null) {
            m_climber.stopMotor();
        }
        if (m_climber2 != null) {
            m_climber2.stopMotor();
        }
        if (climberPub != null) {
            climberPub.close();
        }
    }
}
