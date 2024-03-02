package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.ELEVATOR_ID;
import static frc.robot.Constants.Constants.ELEVATOR_PIVOT_ID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ElevatorDeployCommand;
import frc.robot.Commands.ElevatorExtendCommand;
import frc.robot.Commands.ElevatorRetractCommand;
import frc.robot.Commands.ElevatorStowCommand;

public class Elevator extends SubsystemBase {

    private TalonFX m_elevatorPivot;
    private TalonFX m_elevator;

    private double STOWED_ENCODER_VAL = 39; //Actual Stowed Value: 36.4
    private double DEPLOYED_ENCODER_VAL = 3; //Actual Deploy Value: 0

    private double RETRACT_ENCODER_VAL = 2; //Actual Stowed Value: 0
    private double EXTEND_ENCODER_VAL = 300; //Actual Deploy Value: 320

    private ElevatorStowCommand m_elevatorStowCommand = new ElevatorStowCommand(this, STOWED_ENCODER_VAL);
    private ElevatorDeployCommand m_elevatorDeployCommand = new ElevatorDeployCommand(this, DEPLOYED_ENCODER_VAL);
    private ElevatorExtendCommand m_eElevatorExtendCommand = new ElevatorExtendCommand(this, EXTEND_ENCODER_VAL);
    private ElevatorRetractCommand m_elevatorRetractCommand = new ElevatorRetractCommand(this, RETRACT_ENCODER_VAL);

    public Elevator() {
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
        //m_elevatorPivot.setPosition(0);
        var configs1 = new TalonFXConfiguration();
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevatorPivot.getConfigurator().apply(configs1);

        m_elevator = new TalonFX(ELEVATOR_ID);
        m_elevator.setInverted(false);
        m_elevator.setNeutralMode(NeutralModeValue.Brake);
        //m_elevator.setPosition(0);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_elevator.getConfigurator().apply(configs2);
        // zeroValue = zero;
        // halfValue = half;
        // fullValue = full;
        // name = n;
        // m1EncoderVal = 0;
        // m2EncoderVal = 0;
        // elevatorMovingIn = false;
        // elevatorMovingOut = false;
    }

    /* public Command CommandExtend() {
        return Commands.runOnce(() -> Extend());
    }

    public Command CommandRetract() {
        return Commands.runOnce(() -> Retract());
    }*/

    public Command CommandFullExtend() {
        return Commands.runOnce(() -> m_eElevatorExtendCommand.schedule());
    }

    public Command CommandFullRetract() {
        return Commands.runOnce(() -> m_elevatorRetractCommand.schedule());
    }

    public Command CommandStopExtendRetract() {
        return Commands.runOnce(() -> StopExtendRetract());
    }

    public Command CommandPivotDeploy() {
        return Commands.runOnce(() -> m_elevatorDeployCommand.schedule());
    }

    public Command CommandPivotStow() {
        return Commands.runOnce(() -> m_elevatorStowCommand.schedule());
    }

    public Command CommandPivotStop() {
        return Commands.runOnce(() -> StopPivot());
    }

    public void Extend() {
        m_elevator.set(-.85);
    }

    public void Retract() {
        m_elevator.set(.85);
    }

    public void setExtendRetractMotor(double s) {
        m_elevator.set(s);
    }

    public void StopExtendRetract() {
        m_elevator.set(0);
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

    public double getStowedEncoderValue() {
        return STOWED_ENCODER_VAL;
    }

    public double getDeployVal() {
        return DEPLOYED_ENCODER_VAL;
    }

    public double getExtendEncoderValue() {
        var statusSignal = m_elevator.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public double getPivotEncoderValue() {
        var statusSignal = m_elevatorPivot.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public void zero() {
        m_elevator.setPosition(0);
        m_elevatorPivot.setPosition(0);
    }
}
