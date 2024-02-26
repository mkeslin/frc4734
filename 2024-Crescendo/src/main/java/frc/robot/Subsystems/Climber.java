package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.CLIMBER_LEFT_ID;
import static frc.robot.Constants.Constants.CLIMBER_RIGHT_ID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ClimberExtendCommand;
import frc.robot.Commands.ClimberRetractCommand;

public class Climber extends SubsystemBase {
    private TalonFX m_climber_left;
    private TalonFX m_climber_right;

    private double RETRACT_ENCODER_VAL = -145; //Actual Stowed Value: -142.47
    private double EXTEND_ENCODER_VAL = 2; //Actual Deploy Value: 0

    private ClimberExtendCommand climberExtendCommand = new ClimberExtendCommand(this, EXTEND_ENCODER_VAL);
    private ClimberRetractCommand climberRetractCommand = new ClimberRetractCommand(this, RETRACT_ENCODER_VAL);

    public Climber() {
        m_climber_left = new TalonFX(CLIMBER_LEFT_ID, "Canivore");
        m_climber_left.setInverted(false);
        m_climber_left.setNeutralMode(NeutralModeValue.Brake);
        m_climber_left.setPosition(0);
        var configs1 = new TalonFXConfiguration();
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_climber_left.getConfigurator().apply(configs1);

        m_climber_right = new TalonFX(CLIMBER_RIGHT_ID, "Canivore");
        m_climber_right.setInverted(false);
        m_climber_right.setNeutralMode(NeutralModeValue.Brake);
        m_climber_right.setPosition(0);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_climber_right.getConfigurator().apply(configs2);
    }

    public Command CommandFullExtend() {
        return Commands.runOnce(() -> climberExtendCommand.schedule());
    }

    public Command CommandFullRetract() {
        return Commands.runOnce(() -> climberRetractCommand.schedule());
    }

    public Command CommandStopExtendRetract() {
        return Commands.runOnce(() -> stopClimberMotors());
    }

    public void setClimberMotors(double s) {
        m_climber_left.set(s);
        m_climber_right.set(s);
    }

    public void stopClimberMotors() {
        m_climber_left.set(0);
        m_climber_right.set(0);
    }

    public double getEncoderValue() {
        var statusSignal = m_climber_right.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public void zero() {
        m_climber_left.setPosition(0);
        m_climber_right.setPosition(0);
    }
}
