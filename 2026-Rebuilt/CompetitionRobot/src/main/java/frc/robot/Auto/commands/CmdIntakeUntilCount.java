package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.DeployableIntake;

/**
 * Command to intake until ball count is reached (time-based fallback).
 * 
 * <p>Since PositionTracker does not have a ball count method, this command
 * implements a time-based intake with sensor checking. It runs the intake
 * and checks the PositionTracker's coral-in-tray sensor to detect when a ball
 * is acquired.
 * 
 * <p>Completion conditions:
 * <ul>
 *   <li>Coral-in-tray sensor indicates ball is present</li>
 *   <li>Timeout expires</li>
 * </ul>
 * 
 * @param intake The intake subsystem
 * @param positionTracker The position tracker for sensor reading
 * @param targetCount Target ball count (currently unused, kept for API compatibility)
 * @param timeoutSec Maximum time to intake
 */
public class CmdIntakeUntilCount extends Command {
    private final DeployableIntake intake;
    private final PositionTracker positionTracker;
    private final int targetCount;
    private final double timeoutSec;
    private final Timer timer = new Timer();
    private Command intakeCommand;

    /**
     * Creates a new CmdIntakeUntilCount command.
     * 
     * @param intake The intake subsystem
     * @param positionTracker The position tracker for sensor reading
     * @param targetCount Target ball count (currently unused)
     * @param timeoutSec Maximum time to intake
     * @throws NullPointerException if intake or positionTracker is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdIntakeUntilCount(
            DeployableIntake intake,
            PositionTracker positionTracker,
            int targetCount,
            double timeoutSec) {
        this.intake = Objects.requireNonNull(intake, "intake cannot be null");
        this.positionTracker = Objects.requireNonNull(positionTracker, "positionTracker cannot be null");
        this.targetCount = targetCount;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        // Start intake at forward speed
        // TODO: Replace with actual intake speed constant from IntakeConstants
        double intakeSpeed = 0.5; // Placeholder
        intakeCommand = intake.moveToArbitraryIntakeSpeedCommand(() -> intakeSpeed);
        intakeCommand.initialize();
    }

    @Override
    public void execute() {
        if (intakeCommand != null) {
            intakeCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }

        // Check if coral is in tray (sensor indicates ball present)
        Boolean coralInTray = positionTracker.getCoralInTray();
        if (coralInTray != null && coralInTray) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (intakeCommand != null) {
            intakeCommand.end(interrupted);
        }
        // Stop intake on end
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(intake.resetIntakeSpeedCommand());
    }

    /**
     * Factory method to create a command with default timeout.
     * 
     * @param intake The intake subsystem
     * @param positionTracker The position tracker for sensor reading
     * @param targetCount Target ball count
     * @return A command that intakes until count with default timeout
     */
    public static Command create(DeployableIntake intake, PositionTracker positionTracker, int targetCount) {
        return new CmdIntakeUntilCount(
                intake,
                positionTracker,
                targetCount,
                AutoConstants.DEFAULT_INTAKE_TIMEOUT);
    }
}
