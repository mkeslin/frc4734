package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/**
 * Command to set the shooter shot mode.
 * 
 * <p>This command sets the shot mode for the shooter subsystem. The shot mode
 * represents intent (AUTO_SHOT, SAFE_SHOT) and is used by a separate shot model
 * or calculator to determine RPM values, hood angles, and other tuning parameters.
 * 
 * <p>This is a placeholder command for future shot model integration. The actual
 * implementation will depend on how the shot model is integrated with the shooter.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Shot mode is set (completes immediately, non-blocking)</li>
 * </ul>
 * 
 * @param shooter The shooter subsystem
 * @param mode The shot mode to set
 */
public class CmdSetShotMode extends Command {
    private final Shooter shooter;
    private final ShotMode mode;

    /**
     * Creates a new CmdSetShotMode command.
     * 
     * @param shooter The shooter subsystem
     * @param mode The shot mode to set
     * @throws NullPointerException if shooter or mode is null
     */
    public CmdSetShotMode(Shooter shooter, ShotMode mode) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.mode = Objects.requireNonNull(mode, "mode cannot be null");

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // TODO: Integrate with shot model/calculator when available
        // For now, this is a placeholder that completes immediately
        // Future implementation might look like:
        // shotModel.setMode(mode);
        // shooter.setTargetRpm(shotModel.getRpmForCurrentDistance());
    }

    @Override
    public boolean isFinished() {
        // Non-blocking - completes immediately
        return true;
    }
}
