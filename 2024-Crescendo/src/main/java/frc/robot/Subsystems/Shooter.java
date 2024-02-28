package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ShooterSetAngleCommand;

public class Shooter extends SubsystemBase {

    private TalonFX m_shooterIn;
    private TalonFX m_shooterOutTop;
    private TalonFX m_shooterOutBottom;
    private TalonFX m_shooterPivot;

    public static double MAX_PIVOT_ENCODER_VAL = 8.5; //Actual Max Value: 6.9
    public static double TELEOP_SPEAKER_PIVOT_ENCODER_VAL = 7.25;
    public static double AUTO_SPEAKER_PIVOT_ENCODER_VAL = 7.5;

    private ShooterSetAngleCommand m_shooterSetAngleCommand = new ShooterSetAngleCommand(this, TELEOP_SPEAKER_PIVOT_ENCODER_VAL);

    public Shooter() {
        m_shooterIn = new TalonFX(SHOOTER_IN_ID);
        m_shooterIn.setInverted(false);
        m_shooterIn.setNeutralMode(NeutralModeValue.Brake);
        var configs1 = new TalonFXConfiguration();
        configs1.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_shooterIn.getConfigurator().apply(configs1);

        m_shooterOutTop = new TalonFX(SHOOTER_OUT_TOP_ID);
        m_shooterOutTop.setInverted(false);
        m_shooterOutTop.setNeutralMode(NeutralModeValue.Brake);
        var configs2 = new TalonFXConfiguration();
        configs2.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_shooterOutTop.getConfigurator().apply(configs2);

        m_shooterOutBottom = new TalonFX(SHOOTER_OUT_BOTTOM_ID);
        m_shooterOutBottom.setInverted(false);
        m_shooterOutBottom.setNeutralMode(NeutralModeValue.Brake);
        var configs3 = new TalonFXConfiguration();
        configs3.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_shooterOutBottom.getConfigurator().apply(configs3);

        m_shooterPivot = new TalonFX(SHOOTER_PIVOT_ID);
        m_shooterPivot.setInverted(false);
        m_shooterPivot.setNeutralMode(NeutralModeValue.Brake);
        var configs4 = new TalonFXConfiguration();
        configs4.CurrentLimits = new CurrentLimitsConfigs();
        // configs.CurrentLimits.SupplyCurrentLimit = 20;
        // configs.CurrentLimits.SupplyCurrentLimit = 40;
        m_shooterPivot.getConfigurator().apply(configs4);
        m_shooterPivot.setPosition(0);
    }

    public Command commandShoot() {
        return Commands.runOnce(() -> this.shoot());
    }

    public Command commandShoot(double speed) {
        return Commands.runOnce(() -> this.shoot(speed));
    }

    public Command commandStop() {
        return Commands.runOnce(() -> this.stopShoot());
    }

    public Command commandSetAngle(double a) {
        return Commands.runOnce(() -> {
            m_shooterSetAngleCommand.setTarget(a);
            m_shooterSetAngleCommand.schedule();
        });
    }

    public Command commandIncrementAngle(double a) {
        return Commands.runOnce(() -> {
            //shooterSetAngleCommand.setTarget(getPivotEncoderValue() + a);
            //shooterSetAngleCommand.schedule();
        });
    }

    public void shoot() {
        shoot(1);
    }

    public void shoot(double speed) {
        m_shooterOutTop.set(speed);
        m_shooterOutBottom.set(speed);
        //m_shooterIn.set(-speed);
    }

    public void stopShoot() {
        m_shooterOutTop.set(0);
        m_shooterOutBottom.set(0);
        //m_shooterIn.set(0);
    }

    public double getPivotEncoderValue() {
        var statusSignal = m_shooterPivot.getPosition();
        return statusSignal.getValueAsDouble();
    }

    public void setPivotMotor(double s) {
        m_shooterPivot.set(s);
    }

    public void stopPivot() {
        setPivotMotor(0);
    }

    public void holdPivot() {
        setPivotMotor(0.0275);
    }
}
