package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalStates;
import frc.robot.PositionTracker;
import frc.robot.Utils;
import frc.robot.Constants.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Subsystems.Bases.BaseLinearMechanism;

import static frc.robot.Constants.Constants.IDs.ELEVATOR_1_ID;
import static frc.robot.Constants.Constants.IDs.ELEVATOR_2_ID;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase implements BaseLinearMechanism<ElevatorPosition> {

    private TalonFX m_elevator1;
    private TalonFX m_elevator2;

    private double simVelocity = 0.0;

    private double RETRACT_ENCODER_VAL = 2; //Actual Stowed Value: 0
    // private double EXTEND_L1_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double EXTEND_L2_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double EXTEND_L3_ENCODER_VAL = 300; //Actual Deploy Value: 320
    // private double EXTEND_L4_ENCODER_VAL = 300; //Actual Deploy Value: 320

    private final PositionTracker m_positionTracker;
    // private final MechanismLigament2d ligament;

    // @Log
    private boolean initialized;

//     private ElevatorStowCommand m_elevatorStowCommand = new ElevatorStowCommand(this, STOWED_ENCODER_VAL);
//     private ElevatorDeployCommand m_elevatorDeployCommand = new ElevatorDeployCommand(this, DEPLOYED_ENCODER_VAL);
//     private ElevatorExtendCommand m_eElevatorExtendCommand = new ElevatorExtendCommand(this, EXTEND_ENCODER_VAL);
//     private ElevatorRetractCommand m_elevatorRetractCommand = new ElevatorRetractCommand(this, RETRACT_ENCODER_VAL);

    // private final ElevatorSim elevatorSim = new ElevatorSim(
    //     MOTOR_GEARBOX_REPR,
    //     GEARING,
    //     MASS_KG,
    //     DRUM_RADIUS_METERS,
    //     MIN_HEIGHT_METERS,
    //     MAX_HEIGHT_METERS,
    //     true,
    //     ElevatorPosition.BOTTOM.value);

    public Elevator(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
        positionTracker.setElevatorPositionSupplier(this::getPosition);

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

        m_elevator1 = new TalonFX(ELEVATOR_1_ID);
        // m_elevator1.setInverted(false);
        m_elevator1.setNeutralMode(NeutralModeValue.Brake);
        //m_elevatorPivot.setPosition(0);
        var configs1 = new TalonFXConfiguration();
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        configs1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevator1.getConfigurator().apply(configs1);

        m_elevator2 = new TalonFX(ELEVATOR_2_ID);
        // m_elevator2.setInverted(false);
        m_elevator2.setNeutralMode(NeutralModeValue.Brake);
        //m_elevator.setPosition(0);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        configs1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevator2.getConfigurator().apply(configs2);

        // zeroValue = zero;
        // halfValue = half;
        // fullValue = full;
        // name = n;
        // m1EncoderVal = 0;
        // m2EncoderVal = 0;
        // elevatorMovingIn = false;
        // elevatorMovingOut = false;
    }

//     /* public Command CommandExtend() {
//         return Commands.runOnce(() -> Extend());
//     }

//     public Command CommandRetract() {
//         return Commands.runOnce(() -> Retract());
//     }*/

//     public Command CommandFullExtend() {
//         return Commands.runOnce(() -> m_eElevatorExtendCommand.schedule());
//     }

//     public Command CommandFullRetract() {
//         return Commands.runOnce(() -> m_elevatorRetractCommand.schedule());
//     }

//     public Command CommandStopExtendRetract() {
//         return Commands.runOnce(() -> StopExtendRetract());
//     }

//     public Command CommandPivotDeploy() {
//         return Commands.runOnce(() -> m_elevatorDeployCommand.schedule());
//     }

//     public Command CommandPivotStow() {
//         return Commands.runOnce(() -> m_elevatorStowCommand.schedule());
//     }

//     public Command CommandPivotStop() {
//         return Commands.runOnce(() -> StopPivot());
//     }

//     public void Extend() {
//         m_elevator.set(-.85);
//     }

//     public void Retract() {
//         m_elevator.set(.85);
//     }

//     public void setExtendRetractMotor(double s) {
//         m_elevator.set(s);
//     }

//     public void StopExtendRetract() {
//         m_elevator.set(0);
//     }

//     public void setPivot(double s) {
//         m_elevatorPivot.set(s);
//     }

//     public void PivotOut() {
//         setPivot(0.1);
//     }

//     public void PivotIn() {
//         setPivot(-0.1);
//     }

//     public void StopPivot() {
//         setPivot(0);
//     }

//     public double getStowedEncoderValue() {
//         return STOWED_ENCODER_VAL;
//     }

//     public double getDeployVal() {
//         return DEPLOYED_ENCODER_VAL;
//     }

//     public double getExtendEncoderValue() {
//         var statusSignal = m_elevator.getPosition();
//         return statusSignal.getValueAsDouble();
//     }

//     public double getPivotEncoderValue() {
//         var statusSignal = m_elevatorPivot.getPosition();
//         return statusSignal.getValueAsDouble();
//     }

    // public void zero() {
    //     m_elevator1.setPosition(0);
    //     m_elevator2.setPosition(0);
    // }





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
    // public Pose3d getCarriageComponentPose() {
    //     return new Pose3d(0.14, 0, 0.247 + getPosition(), new Rotation3d());
    // }

    @Override
    public double getPosition() {
        return m_elevator1.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        if (RobotBase.isReal())
            return m_elevator1.getVelocity().getValueAsDouble();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        m_elevator1.setPosition(ElevatorPosition.BOTTOM.value);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), ElevatorConstants.MIN_HEIGHT_METERS, ElevatorConstants.MAX_HEIGHT_METERS);

        if (voltage < 0
                && m_positionTracker.getElevatorPosition() < ElevatorConstants.MOTION_LIMIT
                && m_positionTracker.getArmAngle() < 0) {
            voltage = 0;
        }

        if (!GlobalStates.INITIALIZED.enabled()) {
            voltage = 0.0;
        }

        m_elevator1.setVoltage(voltage);

    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return null;
        // return run(() -> {
        //     feedbackVoltage = pidController.calculate(getPosition());
        //     feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().velocity);
        //     setVoltage(feedbackVoltage + feedforwardVoltage);
        // }).withName("elevator.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        return null;
        // return Commands.sequence(
        //         runOnce(() -> pidController.reset(getPosition())),
        //         runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
        //         moveToCurrentGoalCommand()
        //                 .until(() -> pidController.atGoal()))
        //         .withTimeout(3)
        //         .withName("elevator.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return null;
        // return Commands.sequence(
        //         runOnce(() -> pidController.reset(getPosition())),
        //         runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
        //         moveToCurrentGoalCommand().until(this::atGoal)).withName("elevator.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return null;
        // return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
        //         .withName("elevator.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return null;
        // return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
        //         .withName("elevator.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("elevator.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return null;
        // return runOnce(m_elevator1::stopMotor)
        //         .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
        //         .finallyDo((d) -> {
        //             motor.setIdleMode(IdleMode.kBrake);
        //             pidController.reset(getPosition());
        //         }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        //         .withName("elevator.coastMotorsCommand");
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

    // public boolean atGoal() {
    //     return pidController.atGoal();
    // }
}
