package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/**
 * Command to stop the shooter subsystem.
 * 
 * <p>This command stops the shooter by resetting its speed to zero.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Shooter stop command completes (immediate)</li>
 * </ul>
 * 
 * @param shooter The shooter subsystem
 */
public class CmdStopShooter extends Command {
    private final Shooter shooter;

    /**
     * Creates a new CmdStopShooter command.
     * 
     * @param shooter The shooter subsystem
     * @throws NullPointerException if shooter is null
     */
    public CmdStopShooter(Shooter shooter) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(shooter.resetSpeedCommand());
    }

    @Override
    public boolean isFinished() {
        // Completes immediately after scheduling stop
        return true;
    }
}
