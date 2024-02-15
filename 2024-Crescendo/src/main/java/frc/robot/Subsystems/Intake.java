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
import frc.robot.Commands.IntakeStowCommand;
import frc.robot.Commands.IntakeDeployCommand;

public class Intake extends SubsystemBase {

    private TalonFX m_roller;
    private TalonFX m_shooterIn;
    private boolean stowing;
    private boolean deploying;
    private TalonFX pivot;
    private double pivotEncoderVal;

    private double STOWED_ENCODER_VAL = -0.5;    //Actual Stowed Value: 0
    private double DEPLOYED_ENCODER_VAL = -4.5;    //Actual Deploy Value: -5.175

    private IntakeStowCommand intakeStowCommand = new IntakeStowCommand(this, STOWED_ENCODER_VAL);
    private IntakeDeployCommand intakeDeployCommand = new IntakeDeployCommand(this, DEPLOYED_ENCODER_VAL);
    // private TalonFX wheels2;
    // private PneumaticHub hub;
    // private DoubleSolenoid sol;
    // private Compressor compressor;

    public Intake() {
        m_roller = new TalonFX(INTAKE_ID);
        m_shooterIn = new TalonFX(SHOOTER_IN_ID);
        // hub = new PneumaticHub();
        // sol = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);
        // compressor = new Compressor(1, PneumaticsModuleType.REVPH);

        m_roller.setInverted(false);
        m_roller.setNeutralMode(NeutralModeValue.Brake);
        // talon.configOpenloopRamp(0.15);
        // wheels1.configContinuousCurrentLimit(20);
        // wheels1.configPeakCurrentLimit(40);
        // wheels1.configPeakCurrentDuration(400);
        // wheels1.enableCurrentLimit(true);
        // talon.setName("Intake");

        var configs = new TalonFXConfiguration();
        configs.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_roller.getConfigurator().apply(configs);


        pivot = new TalonFX(INTAKE_PIVOT_ID);
        pivot.setInverted(false);
        pivot.setNeutralMode(NeutralModeValue.Brake);
        pivot.setPosition(0);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        pivot.getConfigurator().apply(configs2);

        pivotEncoderVal = 0;
        stowing = false;
        deploying = false;
    }

    public Command commandStartIn() {
        return Commands.runOnce(() -> this.startIn());
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
        // wheels2.set(1);

		SmartDashboard.putNumber("Intake speed setpoint", 1);
    }

    public void stopRoller() {
        // wheels1.set(ControlMode.PercentOutput, 0);
        // wheels2.set(ControlMode.PercentOutput, 0);

        m_roller.set(0);
        m_shooterIn.set(0);
        // wheels2.set(0);

        SmartDashboard.putNumber("Intake speed setpoint", 0);
    }




    public Command commandStow() {
        return Commands.runOnce(() -> intakeStowCommand.schedule());
    }
    public Command commandDeploy() {
        return Commands.runOnce(() -> intakeDeployCommand.schedule());
    }
    public Command commandStopPivot() {
        return Commands.run(() -> this.stopPivot());
    }

    public double getEncoderValue() {
        var statusSignal = pivot.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public void setPivotMotor(double s) {
        SmartDashboard.putNumber("Encoder Val", getEncoderValue());
        SmartDashboard.putNumber("Speed", s);
        pivot.set(s);
    }

    public void stow() {
        /*stowing = true;
        deploying = false;
        var startEncoder = getEncoderValue();
        var currentEncoder = startEncoder; 
        var distance = 2;
        //pivot.set(.125);
        while(stowing && Math.abs(currentEncoder - startEncoder) < 4) {
            SmartDashboard.putNumber("Intake-Pivot", getEncoderValue());
            if(Math.abs(currentEncoder - startEncoder) > distance) {
                pivot.set(.125);
            } else {
                pivot.set(.25);
            }
            currentEncoder = getEncoderValue();
        }
        pivot.set(0);
        stowing = false;*/
    }
    public void deploy() {
        /*stowing = false;
        deploying = true;
        var startEncoder = getEncoderValue();
        var currentEncoder = startEncoder; 
        var distance = 0.5;
        //pivot.set(-.125);
        while(deploying && Math.abs(currentEncoder - startEncoder) < 1.5) {
            SmartDashboard.putNumber("Intake-Pivot", getEncoderValue());
            if(Math.abs(currentEncoder - startEncoder) > distance) {
                pivot.set(-.125);
            } else {
                pivot.set(-.25);
            }
            currentEncoder = getEncoderValue();
        }
        pivot.set(0);
        deploying = false;*/
    }
    public void stopPivot()
    {
        pivot.set(0);
    }


    // @Override
	// public void periodic() {
	// 	// setSpeed(Robot.m_oi.getIntakeSpeed());
	// 	// System.out.println("speed " + Robot.m_oi.getIntakeSpeed());
	// }

	// @Override
	// public void initDefaultCommand() {
	// 	// Set the default command for a subsystem here.
	// 	setDefaultCommand(new IntakeTelop());
	// }
}
