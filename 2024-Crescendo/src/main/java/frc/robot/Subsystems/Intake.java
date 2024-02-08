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

public class Intake extends SubsystemBase {

    private TalonFX roller;

    private TalonFX pivot;
    private double pivotEncoderVal;

    // private TalonFX wheels2;
    // private PneumaticHub hub;
    // private DoubleSolenoid sol;
    // private Compressor compressor;

    public Intake() {
        roller = new TalonFX(INTAKEID);
        // hub = new PneumaticHub();
        // sol = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);
        // compressor = new Compressor(1, PneumaticsModuleType.REVPH);

        roller.setInverted(false);
        roller.setNeutralMode(NeutralModeValue.Brake);
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
        roller.getConfigurator().apply(configs);


        pivot = new TalonFX(INTAKEPIVOTID);
        pivot.setInverted(false);
        pivot.setNeutralMode(NeutralModeValue.Brake);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        pivot.getConfigurator().apply(configs2);

        pivotEncoderVal = 0;
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
        return roller.get();
    }

    public void startIn() {
        roller.set(-.25);
        // wheels2.set(1);

		SmartDashboard.putNumber("Intake speed setpoint", 1);
    }

    public void stopRoller() {
        // wheels1.set(ControlMode.PercentOutput, 0);
        // wheels2.set(ControlMode.PercentOutput, 0);

        roller.set(0);
        // wheels2.set(0);

        SmartDashboard.putNumber("Intake speed setpoint", 0);
    }




    public Command commandStow() {
        return Commands.runOnce(() -> this.stow());
    }
    public Command commandDeploy() {
        return Commands.runOnce(() -> this.deploy());
    }
    public Command commandStopPivot() {
        return Commands.runOnce(() -> this.stopPivot());
    }

    public double getEncoderValue() {
        var statusSignal = pivot.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public void stow() {
        var startEncoder = getEncoderValue();
        var currentEncoder = startEncoder; 
        pivot.set(.125);
        while(Math.abs(currentEncoder - startEncoder) < 4) {
            SmartDashboard.putNumber("Intake-Pivot", getEncoderValue());
            currentEncoder = getEncoderValue();
        }
        pivot.set(0);
    }
    public void deploy() {
        var startEncoder = getEncoderValue();
        var currentEncoder = startEncoder; 
        pivot.set(-.125);
        while(Math.abs(currentEncoder - startEncoder) < 1) {
            SmartDashboard.putNumber("Intake-Pivot", getEncoderValue());
            currentEncoder = getEncoderValue();
        }
        pivot.set(0);
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
