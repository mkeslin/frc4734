package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.CANIds.CAN_BUS;
import static frc.robot.Constants.CANIds.CLIMBER;
import static frc.robot.Constants.CANIds.CLIMBER_JAW_LEFT;
import static frc.robot.Constants.CANIds.CLIMBER_JAW_RIGHT;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.TelemetryCalcs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.ClimberConstants.JawPosition;
import frc.robot.Subsystems.Bases.BaseSingleJointedArm;

/**
 * Climber subsystem that controls the robot's climbing mechanism: a lift motor and left/right jaw motors.
 * Lift: one TalonFX with MotionMagic for position-based movement (ClimberPosition).
 * Jaws: two independent TalonFX motors, each with open/closed positions (JawPosition).
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
 * @see JawPosition
 * @see RobotState
 */
public class Climber extends SubsystemBase implements BaseSingleJointedArm<ClimberPosition> {
    private final DoublePublisher climberPub = TelemetryCalcs.createMechanismsPublisher("Climber Position");

    private TalonFX m_climber;
    private TalonFX m_jawLeft;
    private TalonFX m_jawRight;

    private PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;
    // private final Supplier<Pose3d> carriagePoseSupplier;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final MotionMagicVoltage m_jawLeftRequest = new MotionMagicVoltage(0);
    private final MotionMagicVoltage m_jawRightRequest = new MotionMagicVoltage(0);

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

        // Flipped so negative position command drives motor in desired direction for ascent
        talonFxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_climber = new TalonFX(CLIMBER, CAN_BUS);
        m_climber.setNeutralMode(NeutralModeValue.Brake);
        m_climber.getConfigurator().apply(talonFxConfigs);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(ClimberConstants.SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(ClimberConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(ClimberConstants.STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(ClimberConstants.STATOR_CURRENT_LIMIT_ENABLE);
        ClosedLoopRampsConfigs closedLoopRamps = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(ClimberConstants.CLOSED_LOOP_VOLTAGE_RAMP_PERIOD_SEC);
        VoltageConfigs voltage = new VoltageConfigs()
                .withPeakForwardVoltage(ClimberConstants.PEAK_FORWARD_VOLTAGE)
                .withPeakReverseVoltage(ClimberConstants.PEAK_REVERSE_VOLTAGE);
        m_climber.getConfigurator().apply(currentLimits);
        m_climber.getConfigurator().apply(closedLoopRamps);
        m_climber.getConfigurator().apply(voltage);

        // Jaws: position control (open/closed) with MotionMagic
        var jawConfigs = new TalonFXConfiguration();
        jawConfigs.Slot0.kP = ClimberConstants.JAW_KP;
        jawConfigs.Slot0.kI = ClimberConstants.JAW_KI;
        jawConfigs.Slot0.kD = ClimberConstants.JAW_KD;
        jawConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.JAW_MOTION_MAGIC_CRUISE_VELOCITY;
        jawConfigs.MotionMagic.MotionMagicAcceleration = ClimberConstants.JAW_MOTION_MAGIC_ACCELERATION;
        jawConfigs.MotionMagic.MotionMagicJerk = ClimberConstants.JAW_MOTION_MAGIC_JERK;
        jawConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs jawCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(ClimberConstants.JAW_SUPPLY_CURRENT_LIMIT_AMPS))
                .withSupplyCurrentLimitEnable(ClimberConstants.SUPPLY_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(ClimberConstants.JAW_STATOR_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(ClimberConstants.STATOR_CURRENT_LIMIT_ENABLE);

        m_jawLeft = new TalonFX(CLIMBER_JAW_LEFT, CAN_BUS);
        m_jawLeft.setNeutralMode(NeutralModeValue.Brake);
        m_jawLeft.getConfigurator().apply(jawConfigs);
        m_jawLeft.getConfigurator().apply(jawCurrentLimits);
        m_jawLeft.getConfigurator().apply(closedLoopRamps);
        m_jawLeft.getConfigurator().apply(voltage);

        m_jawRight = new TalonFX(CLIMBER_JAW_RIGHT, CAN_BUS);
        m_jawRight.setNeutralMode(NeutralModeValue.Brake);
        m_jawRight.getConfigurator().apply(jawConfigs);
        // Right jaw: positive rotation = clamping (mirror of left)
        m_jawRight.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        m_jawRight.getConfigurator().apply(jawCurrentLimits);
        m_jawRight.getConfigurator().apply(closedLoopRamps);
        m_jawRight.getConfigurator().apply(voltage);

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
        m_jawLeft.setPosition(ClimberConstants.LEFT_JAW_CLOSED_ROTATIONS);
        m_jawRight.setPosition(ClimberConstants.RIGHT_JAW_CLOSED_ROTATIONS);
        initialized = true;
    }

    /** Left jaw position in rotations. */
    public double getLeftJawPosition() {
        return m_jawLeft.getPosition().getValueAsDouble();
    }

    /** Right jaw position in rotations. */
    public double getRightJawPosition() {
        return m_jawRight.getPosition().getValueAsDouble();
    }

    private static double jawPositionError(double current, double goal) {
        return Math.abs(current - goal);
    }

    /**
     * Sets the lift goal position (MotionMagic). Used by climb/descend commands that drive the lift from their execute().
     * Only applies when robot is initialized; otherwise stops the lift motor.
     */
    public void setLiftGoalPosition(double positionRotations) {
        if (RobotState.getInstance().isInitialized()) {
            m_climber.setControl(m_request.withPosition(positionRotations));
        } else {
            m_climber.stopMotor();
        }
    }

    /**
     * Stops the lift motor. Used when a climb/descend command is interrupted (e.g. button released).
     */
    public void stopLift() {
        m_climber.stopMotor();
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
            } else {
                m_climber.stopMotor();
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

    // ---- Jaw commands (independent left/right, open/closed) ----

    private Command moveLeftJawToCommand(JawPosition position) {
        double goal = position == JawPosition.OPEN
                ? ClimberConstants.LEFT_JAW_OPEN_ROTATIONS
                : ClimberConstants.LEFT_JAW_CLOSED_ROTATIONS;
        return run(() -> {
            if (RobotState.getInstance().isInitialized()) {
                m_jawLeft.setControl(m_jawLeftRequest.withPosition(goal));
            } else {
                m_jawLeft.stopMotor();
            }
        })
                .until(() -> jawPositionError(getLeftJawPosition(), goal) < ClimberConstants.JAW_POSITION_TOLERANCE_ROTATIONS)
                .withTimeout(5)
                .withName("climber.moveLeftJawTo");
    }

    private Command moveRightJawToCommand(JawPosition position) {
        double goal = position == JawPosition.OPEN
                ? ClimberConstants.RIGHT_JAW_OPEN_ROTATIONS
                : ClimberConstants.RIGHT_JAW_CLOSED_ROTATIONS;
        return run(() -> {
            if (RobotState.getInstance().isInitialized()) {
                m_jawRight.setControl(m_jawRightRequest.withPosition(goal));
            } else {
                m_jawRight.stopMotor();
            }
        })
                .until(() -> jawPositionError(getRightJawPosition(), goal) < ClimberConstants.JAW_POSITION_TOLERANCE_ROTATIONS)
                .withTimeout(5)
                .withName("climber.moveRightJawTo");
    }

    /** Command to open the left jaw. */
    public Command openLeftJawCommand() {
        return moveLeftJawToCommand(JawPosition.OPEN).withName("climber.openLeftJaw");
    }

    /** Command to close the left jaw. */
    public Command closeLeftJawCommand() {
        return moveLeftJawToCommand(JawPosition.CLOSED).withName("climber.closeLeftJaw");
    }

    /** Command to open the right jaw. */
    public Command openRightJawCommand() {
        return moveRightJawToCommand(JawPosition.OPEN).withName("climber.openRightJaw");
    }

    /** Command to close the right jaw. */
    public Command closeRightJawCommand() {
        return moveRightJawToCommand(JawPosition.CLOSED).withName("climber.closeRightJaw");
    }

    /** Command to open both jaws (runs in parallel). */
    public Command openBothJawsCommand() {
        return Commands.parallel(openLeftJawCommand(), openRightJawCommand()).withName("climber.openBothJaws");
    }

    /** Command to close both jaws (runs in parallel). */
    public Command closeBothJawsCommand() {
        return Commands.parallel(closeLeftJawCommand(), closeRightJawCommand()).withName("climber.closeBothJaws");
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
                    m_jawLeft.stopMotor();
                    m_jawRight.stopMotor();
                })
                .andThen(() -> {
                    m_climber.setNeutralMode(NeutralModeValue.Coast);
                    m_jawLeft.setNeutralMode(NeutralModeValue.Coast);
                    m_jawRight.setNeutralMode(NeutralModeValue.Coast);
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
        if (m_jawLeft != null) {
            m_jawLeft.stopMotor();
        }
        if (m_jawRight != null) {
            m_jawRight.stopMotor();
        }
        if (climberPub != null) {
            climberPub.close();
        }
    }
}
