package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.IntakeDeployCommand;
import frc.robot.Commands.IntakeStowCommand;

public class Intake extends SubsystemBase {

    private TalonFX m_intakeTop;
    private TalonFX m_intakeBottom;
    // private TalonFX m_pivot;

    // private DigitalInput m_intakeSensor = new DigitalInput(INTAKE_SENSOR);

    private double STOWED_ENCODER_VAL = 0.5; //Actual Stowed Value: 0
    private double DEPLOYED_ENCODER_VAL = 4.5; //Actual Deploy Value: -5.175

    public double getStowedEncoderValue() { return STOWED_ENCODER_VAL; }
    public double getDeployedEncoderValue() { return DEPLOYED_ENCODER_VAL; }

    private IntakeStowCommand m_intakeStowCommand = new IntakeStowCommand(this, STOWED_ENCODER_VAL);
    private IntakeDeployCommand m_intakeDeployCommand = new IntakeDeployCommand(this, DEPLOYED_ENCODER_VAL);

    public Intake() {
        m_intakeTop = new TalonFX(INTAKE_TOP_ID);
        m_intakeBottom = new TalonFX(INTAKE_BOTTOM_ID);
        // m_shooterIn = new TalonFX(SHOOTER_IN_ID);

        m_intakeTop.setInverted(false);
        m_intakeTop.setNeutralMode(NeutralModeValue.Brake);
        // talon.configOpenloopRamp(0.15);
        // wheels1.configContinuousCurrentLimit(20);
        // wheels1.configPeakCurrentLimit(40);
        // wheels1.configPeakCurrentDuration(400);
        // wheels1.enableCurrentLimit(true);
        var configs = new TalonFXConfiguration();
        configs.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        configs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
        m_intakeTop.getConfigurator().apply(configs);

        m_intakeBottom.setInverted(false);
        m_intakeBottom.setNeutralMode(NeutralModeValue.Brake);
        // talon.configOpenloopRamp(0.15);
        // wheels1.configContinuousCurrentLimit(20);
        // wheels1.configPeakCurrentLimit(40);
        // wheels1.configPeakCurrentDuration(400);
        // wheels1.enableCurrentLimit(true);
        var configs_Bottom = new TalonFXConfiguration();
        configs_Bottom.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        configs_Bottom.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
        m_intakeBottom.getConfigurator().apply(configs_Bottom);

        // m_pivot = new TalonFX(INTAKE_PIVOT_ID);
        // m_pivot.setInverted(false);
        // m_pivot.setNeutralMode(NeutralModeValue.Brake);
        // // m_pivot.setPosition(0);
        // var configs2 = new TalonFXConfiguration();
        // configs2.CurrentLimits = new CurrentLimitsConfigs();
        // // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // // configs.CurrentLimits.SupplyCurrentLimit = 40;
        // m_pivot.getConfigurator().apply(configs2);
    }

    public Command commandStartIn() {
        return Commands.runOnce(() -> this.startIn(-.3));
    }

    public Command commandStartOut() {
        return Commands.runOnce(() -> this.startOut());
    }

    public Command commandStop() {
        return Commands.runOnce(() -> this.stopRoller());
    }

    public boolean isOn() {
        return getSpeed() > 0;
    }

    public double getSpeed() {
        return m_intakeTop.get();
    }

    public void startIn(double s) {
        m_intakeTop.set(s);
        m_intakeBottom.set(-s);
    }

    public void startOut() {
        m_intakeTop.set(.25);
        m_intakeBottom.set(-.25);
    }

    public void stopRoller() {
        m_intakeTop.set(0);
        m_intakeBottom.set(0);

        SmartDashboard.putNumber("Intake speed setpoint", 0);
    }

    public Command commandStow() {
        return Commands.runOnce(() -> m_intakeStowCommand.schedule());
    }

    public Command commandDeploy() {
        return Commands.runOnce(() -> m_intakeDeployCommand.schedule());
    }

    public Command commandStopPivot() {
        return Commands.run(() -> this.stopPivot());
    }

    public double getEncoderValue() {
        // var statusSignal = m_pivot.getPosition();
        // return statusSignal.getValueAsDouble();
        return 0;
    }

    public void setPivotMotor(double s) {
        // SmartDashboard.putNumber("Encoder Val", getEncoderValue());
        // SmartDashboard.putNumber("Speed", s);
        // m_pivot.set(s);
    }

    public void stopPivot() {
        // m_pivot.set(0);
    }

    // @Override
    // public void periodic() {}

    // @Override
    // public void initDefaultCommand() {}

    public boolean noteIsSeen() {
        // return !m_intakeSensor.get();
        return true;
    }

    public void zero() {
        // m_pivot.setPosition(0);
    }
}
