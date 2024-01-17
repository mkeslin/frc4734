package frc.robot.Subsystems;

import static frc.robot.Constants.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX wheels1;
    private TalonFX wheels2;

    public Shooter() {
        wheels1 = new TalonFX(INTAKELEFTID);
        wheels2 = new TalonFX(INTAKERIGHTID);
    }

    public Command commandShoot() {
        return Commands.run(() -> this.shoot());
    }

    public Command commandStop() {
        return Commands.runOnce(() -> this.stop());
    }

    public void shoot() {
        wheels1.set(1);
        wheels2.set(1);
    }

    public void stop() {
        wheels1.set(0);
        wheels2.set(0);
    }
}
