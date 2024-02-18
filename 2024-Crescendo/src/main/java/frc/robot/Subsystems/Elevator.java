package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.ELEVATOR_LEFT_ID;
import static frc.robot.Constants.Constants.ELEVATOR_PIVOT_ID;
import static frc.robot.Constants.Constants.ELEVATOR_RIGHT_ID;
import static frc.robot.Constants.Constants.INTAKE_PIVOT_ID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ElevatorStowCommand;
import frc.robot.Commands.ElevatorDeployCommand;
import frc.robot.Commands.ElevatorExtendCommand;
import frc.robot.Commands.ElevatorRetractCommand;

// public enum ElevatorDirection {
//   ELEVATOR_UP,      // Elevator is going up
//   ELEVATOR_DOWN,    // Elevator is going down
//   ELEVATOR_HOLD     // Elevator is being held
// }

public class Elevator extends SubsystemBase {

    private TalonFX m_elevatorPivot;
    private TalonFX m_elevatorLeft;
    private TalonFX m_elevatorRight;
    private boolean elevatorMovingIn, elevatorMovingOut;
    private int zeroValue, halfValue, fullValue;
    private double m1EncoderVal, m2EncoderVal;
    private String name;

    private double STOWED_ENCODER_VAL = 15; //Actual Stowed Value: 15.97
    private double DEPLOYED_ENCODER_VAL = -22; //Actual Deploy Value: -20.45
    private double RETRACT_ENCODER_VAL = 3; //Actual Stowed Value: 0
    private double EXTEND_ENCODER_VAL = 190; //Actual Deploy Value: -198

    private ElevatorStowCommand elevatorStowCommand = new ElevatorStowCommand(this, STOWED_ENCODER_VAL);
    private ElevatorDeployCommand elevatorDeployCommand = new ElevatorDeployCommand(this, DEPLOYED_ENCODER_VAL);
    private ElevatorExtendCommand elevatorExtendCommand = new ElevatorExtendCommand(this, EXTEND_ENCODER_VAL);
    private ElevatorRetractCommand elevatorRetractCommand = new ElevatorRetractCommand(this, RETRACT_ENCODER_VAL);

    public Elevator() {
        // public Elevator(String n, int id1, int id2, int zero, int half, int full) {
        // motor1 = new TalonFX(id1);
        // motor2 = new TalonFX(id2);
        // motor1.setPosition(0);
        // motor2.setPosition(0);
        // motor1.setNeutralMode(NeutralModeValue.Brake);
        // motor2.setNeutralMode(NeutralModeValue.Brake);

        // // in init function
        // var talonFXConfigs = new TalonFXConfiguration();

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

        m_elevatorPivot = new TalonFX(ELEVATOR_PIVOT_ID);
        m_elevatorPivot.setInverted(false);
        m_elevatorPivot.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorPivot.setPosition(0);
        var configs1 = new TalonFXConfiguration();
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevatorPivot.getConfigurator().apply(configs1);

        m_elevatorLeft = new TalonFX(ELEVATOR_LEFT_ID);
        m_elevatorLeft.setInverted(false);
        m_elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorLeft.setPosition(0);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevatorLeft.getConfigurator().apply(configs2);

        m_elevatorRight = new TalonFX(ELEVATOR_RIGHT_ID);
        m_elevatorRight.setInverted(false);
        m_elevatorRight.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorRight.setPosition(0);
        var configs3 = new TalonFXConfiguration();
        configs3.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevatorRight.getConfigurator().apply(configs3);

        // zeroValue = zero;
        // halfValue = half;
        // fullValue = full;
        // name = n;
        m1EncoderVal = 0;
        m2EncoderVal = 0;
        elevatorMovingIn = false;
        elevatorMovingOut = false;
    }

    public Command CommandExtend() {
        return Commands.runOnce(() -> Extend());
    }

    public Command CommandRetract() {
        return Commands.runOnce(() -> Retract());
    }

    public Command CommandFullExtend() {
        return Commands.runOnce(() -> elevatorExtendCommand.schedule());
    }

    public Command CommandFullRetract() {
        return Commands.runOnce(() -> elevatorRetractCommand.schedule());
    }

    public Command CommandStopExtendRetract() {
        return Commands.runOnce(() -> StopExtendRetract());
    }

    public Command CommandPivotDeploy() {
        return Commands.runOnce(() -> elevatorDeployCommand.schedule());
    }

    public Command CommandPivotStow() {
        return Commands.runOnce(() -> elevatorStowCommand.schedule());
    }

    public Command CommandPivotStop() {
        return Commands.runOnce(() -> StopPivot());
    }

    public void Extend() {
        m_elevatorLeft.set(-.85);
        m_elevatorRight.set(-.85);
    }

    public void Retract() {
        m_elevatorLeft.set(.85);
        m_elevatorRight.set(.85);
    }

    public void setExtendRetractMotor(double s) {
        m_elevatorLeft.set(s);
        m_elevatorRight.set(s);
    }

    public void StopExtendRetract() {
        m_elevatorLeft.set(0);
        m_elevatorRight.set(0);
    }

    public void setPivot(double s) {
        m_elevatorPivot.set(s);
    }

    public void PivotOut() {
        setPivot(0.1);
    }

    public void PivotIn() {
        setPivot(-0.1);
    }

    public void StopPivot() {
        setPivot(0);
    }

    public double getExtendEncoderValue() {
        var statusSignal = m_elevatorLeft.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public double getPivotEncoderValue() {
        var statusSignal = m_elevatorPivot.getPosition();
        return statusSignal.getValueAsDouble();
    }
    // @Override
    // public void periodic() {
    //     // This method will be called once per scheduler run
    // }

    // public void movePositive() {
    //     SmartDashboard.putNumber(name + "Elevator", getElevatorEncoderValue());

    //     elevatorMovingOut = true;
    //     elevatorMovingIn = false;

    //     // create a Motion Magic request, voltage output, slot 0 configs
    //     var request = new MotionMagicVoltage(0).withSlot(0);

    //     // set position to 10 rotations
    //     motor1.setControl(request.withPosition(10));
    //     motor2.setControl(request.withPosition(10));
    //     // if (Math.abs(getElevatorEncoderValue()) < Math.abs(halfValue)) {
    //     //     motor1.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.25);
    //     //     motor2.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.25);
    //     // } else if (Math.abs(getElevatorEncoderValue()) < Math.abs(fullValue)) {
    //     //     motor1.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.1);
    //     //     motor2.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.1);
    //     // } else {
    //     //     setZero(fullValue);
    //     //     elevatorMovingOut = false;
    //     // }
    // }

    // public void moveNegative() {
    //     SmartDashboard.putNumber(name + "Elevator", getElevatorEncoderValue());

    //     elevatorMovingOut = false;
    //     elevatorMovingIn = true;

    //     // create a Motion Magic request, voltage output, slot 0 configs
    //     var request = new MotionMagicVoltage(0).withSlot(0);

    //     // set position to 10 rotations
    //     motor1.setControl(request.withPosition(10));
    //     motor2.setControl(request.withPosition(10));
    //     // if (Math.abs(getElevatorEncoderValue()) > Math.abs(halfValue)) {
    //     //     motor1.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.25);
    //     //     motor2.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.25);
    //     // } else if (Math.abs(getElevatorEncoderValue()) > Math.abs(zeroValue)) {
    //     //     motor1.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.1);
    //     //     motor2.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.1);
    //     // } else {
    //     //     setZero(motor1.getSelectedSensorPosition());
    //     //     elevatorMovingIn = false;
    //     // }
    // }

    // public void setZero(double z) {
    //     m1EncoderVal = z;
    //     m2EncoderVal = z;
    // }

    // public void zero() {
    //     // if (Math.abs(m1EncoderVal - getElevatorEncoderValue()) > 200 && m1EncoderVal > 40000) {
    //     //     motor1.set(
    //     //         ControlMode.MotionMagic,
    //     //         m1EncoderVal,
    //     //         DemandType.ArbitraryFeedForward,
    //     //         0.069
    //     //     );
    //     //     motor2.set(
    //     //         ControlMode.MotionMagic,
    //     //         m2EncoderVal,
    //     //         DemandType.ArbitraryFeedForward,
    //     //         0.069
    //     //     );
    //     // } else {
    //     //     motor1.set(ControlMode.PercentOutput, 0);
    //     //     motor2.set(ControlMode.PercentOutput, 0);
    //     // }
    //     SmartDashboard.putNumber(name + "Elevator", getElevatorEncoderValue());
    //     SmartDashboard.putNumber(name + "Magic", m1EncoderVal);
    // }

    // public double getElevatorEncoderValue() {
    //     // return motor1.getSelectedSensorPosition();
    //     return 0;
    // }

    // public boolean getElevatorMovingIn() {
    //     return elevatorMovingIn;
    // }

    // public boolean getElevatorMovingOut() {
    //     return elevatorMovingOut;
    // }
}
