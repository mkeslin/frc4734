package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX m_shooterIn;
    private TalonFX m_shooterOutTop;
    private TalonFX m_shooterOutBottom;
    private TalonFX m_shooterPivot;

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


        // roller.getConfigurator().apply(new Config);
    }

    public Command commandShoot() {
        return Commands.runOnce(() -> this.shoot());
    }

    public Command commandShoot(double speed) {
        return Commands.runOnce(() -> this.shoot(speed));
    }

    public Command commandStop() {
        return Commands.runOnce(() -> this.stop());
    }

    public void shoot() {
        shoot(.85);
    }

    public void shoot(double speed) {
        m_shooterOutTop.set(speed);
        m_shooterOutBottom.set(speed);
        m_shooterIn.set(-speed);
    }

    public void stop() {
        m_shooterOutTop.set(0);
        m_shooterOutBottom.set(0);
        m_shooterIn.set(0);
    }
}
