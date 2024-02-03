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

    // private TalonFX wheels2;
    // private PneumaticHub hub;
    // private DoubleSolenoid sol;
    // private Compressor compressor;

    public Intake() {
        roller = new TalonFX(INTAKEID);
        // wheels2 = new TalonFX(INTAKERIGHTID);
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
    }

    public Command commandStartIn() {
        return Commands.runOnce(() -> this.startIn());
    }

    // public Command commandStartOut() {
    //     return Commands.runOnce(() -> this.startOut());
    // }

    public Command commandStop() {
        return Commands.runOnce(() -> this.stop());
    }

    // public void open() {
    //     // sol.set(Value.kReverse);
    //     // SmartDashboard.putBoolean("Comp", compressor.isEnabled());
    // }

    // public void close() {
    //     // sol.set(Value.kForward);
    // }

    // public void wheelsIn() {
    //     // wheels1.set(ControlMode.PercentOutput, -0.25);
    //     // wheels2.set(ControlMode.PercentOutput, 0.25);
    // }

    // public void wheelsOut() {
    //     // wheels1.set(ControlMode.PercentOutput, 0.25);
    //     // wheels2.set(ControlMode.PercentOutput, -0.25);
    // }

    public boolean isOn() {
        return getSpeed() > 0;
    }

    public double getSpeed() { 
        return roller.get();
    }

    public void startIn() {
        roller.set(-.5);
        // wheels2.set(1);

		SmartDashboard.putNumber("Intake speed setpoint", 1);
    }

    // public void startOut() {
    //     wheels1.set(-1);
    //     // wheels2.set(-1);
    // }

    public void stop() {
        // wheels1.set(ControlMode.PercentOutput, 0);
        // wheels2.set(ControlMode.PercentOutput, 0);

        roller.set(0);
        // wheels2.set(0);

        SmartDashboard.putNumber("Intake speed setpoint", 0);
    }
    // public void enableCompressor() {
    //     // compressor.enableDigital();
    //     // hub.enableCompressorAnalog(50, 120);
    // }

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
