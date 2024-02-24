package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.IntakeDeployCommand;
import frc.robot.Commands.IntakeStowCommand;

public class Intake extends SubsystemBase {

    private TalonFX m_roller;
    private TalonFX m_shooterIn;
    private TalonFX m_pivot;

    private DigitalInput m_intakeSensor = new DigitalInput(INTAKE_SENSOR);

    private double STOWED_ENCODER_VAL = -0.5; //Actual Stowed Value: 0
    private double DEPLOYED_ENCODER_VAL = -4.5; //Actual Deploy Value: -5.175

    private IntakeStowCommand m_intakeStowCommand = new IntakeStowCommand(this, STOWED_ENCODER_VAL);
    private IntakeDeployCommand m_intakeDeployCommand = new IntakeDeployCommand(this, DEPLOYED_ENCODER_VAL);

    public Intake() {
        m_roller = new TalonFX(INTAKE_ID);
        m_shooterIn = new TalonFX(SHOOTER_IN_ID);

        m_roller.setInverted(false);
        m_roller.setNeutralMode(NeutralModeValue.Brake);
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
        m_roller.getConfigurator().apply(configs);

        m_pivot = new TalonFX(INTAKE_PIVOT_ID);
        m_pivot.setInverted(false);
        m_pivot.setNeutralMode(NeutralModeValue.Brake);
        m_pivot.setPosition(0);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_pivot.getConfigurator().apply(configs2);
    }

    public Command commandStartIn() {
        return Commands.runOnce(() -> this.startIn());
    }

    public Command commandStartOut() {
        return Commands.runOnce(() -> this.startOut());
    }

    public Command commandStopRoller() {
        return Commands.runOnce(() -> this.stopRoller());
    }

    public boolean isOn() {
        return getSpeed() > 0;
    }

    public double getSpeed() {
        return m_roller.get();
    }

    public void startIn() {
        m_roller.set(-.55);
        m_shooterIn.set(-.55);

        SmartDashboard.putNumber("Intake speed setpoint", 1);
    }

    public void startOut() {
        m_roller.set(.25);
        m_shooterIn.set(.25);
    }

    public void stopRoller() {
        m_roller.set(0);
        m_shooterIn.set(0);

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
        var statusSignal = m_pivot.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public void setPivotMotor(double s) {
        SmartDashboard.putNumber("Encoder Val", getEncoderValue());
        SmartDashboard.putNumber("Speed", s);
        m_pivot.set(s);
    }

    public void stopPivot() {
        m_pivot.set(0);
    }

    // @Override
    // public void periodic() {}

    // @Override
    // public void initDefaultCommand() {}

    public boolean noteIsSeen() {
        return !m_intakeSensor.get();
    }
}
