package frc.robot.Auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command to stop multiple subsystems.
 * 
 * <p>This command stops all provided subsystems (drivetrain, shooter, feeder, intake).
 * All parameters are optional (nullable) - only non-null subsystems will be stopped.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>All non-null subsystems are stopped (completes immediately)</li>
 * </ul>
 * 
 * @param drivetrain The drivetrain subsystem (optional)
 * @param shooter The shooter subsystem (optional)
 * @param feeder The feeder subsystem (optional)
 * @param intake The intake subsystem (optional)
 */
public class CmdStopAll extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Feeder feeder;
    private final DeployableIntake intake;

    /**
     * Creates a new CmdStopAll command.
     * 
     * @param drivetrain The drivetrain subsystem (can be null)
     * @param shooter The shooter subsystem (can be null)
     * @param feeder The feeder subsystem (can be null)
     * @param intake The intake subsystem (can be null)
     */
    public CmdStopAll(
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        
        // Add requirements only for non-null subsystems
        if (drivetrain != null) addRequirements(drivetrain);
        if (shooter != null) addRequirements(shooter);
        if (feeder != null) addRequirements(feeder);
        if (intake != null) addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (drivetrain != null) {
            drivetrain.stop();
        }
        if (shooter != null) {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(shooter.resetSpeedCommand());
        }
        if (feeder != null) {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(feeder.resetSpeedCommand());
        }
        if (intake != null) {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(intake.resetIntakeSpeedCommand());
        }
    }

    @Override
    public boolean isFinished() {
        // Completes immediately after stopping all subsystems
        return true;
    }

    /**
     * Factory method to create a command that stops all provided subsystems.
     * 
     * @param drivetrain The drivetrain subsystem (can be null)
     * @param shooter The shooter subsystem (can be null)
     * @param feeder The feeder subsystem (can be null)
     * @param intake The intake subsystem (can be null)
     * @return A command that stops all non-null subsystems
     */
    public static Command create(
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake) {
        return new CmdStopAll(drivetrain, shooter, feeder, intake);
    }
}
