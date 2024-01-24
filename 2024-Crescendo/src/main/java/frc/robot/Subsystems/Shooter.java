package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX rollerBottom;
    private TalonFX rollerTop;

    public Shooter() {
        rollerBottom = new TalonFX(SHOOTERBOTTOMID);
        rollerTop = new TalonFX(SHOOTERTOPID);
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
        shoot(.55);
    }

    public void shoot(double speed) {
        rollerBottom.set(-speed);
        rollerTop.set(speed);
    }

    public void stop() {
        rollerBottom.set(0);
        rollerTop.set(0);
    }
}
