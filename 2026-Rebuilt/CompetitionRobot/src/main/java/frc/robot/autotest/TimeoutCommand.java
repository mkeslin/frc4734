package frc.robot.autotest;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that sets an AtomicReference to TIMEOUT when it finishes.
 * Used in raceWith() pattern for deterministic timeout detection.
 * 
 * <p>This command waits for the specified timeout duration, then sets
 * the endReasonRef to TIMEOUT and finishes. When raced with another
 * command, whichever finishes first wins, allowing deterministic
 * detection of timeout vs success.
 */
public class TimeoutCommand extends Command {
    private final double timeoutSec;
    private final AtomicReference<CommandEndReason> endReasonRef;
    private final Timer timer = new Timer();

    /**
     * Creates a new TimeoutCommand.
     * 
     * @param timeoutSec The timeout duration in seconds
     * @param endReasonRef AtomicReference that will be set to TIMEOUT when this command finishes
     * @throws NullPointerException if endReasonRef is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public TimeoutCommand(double timeoutSec, AtomicReference<CommandEndReason> endReasonRef) {
        this.timeoutSec = timeoutSec;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.endReasonRef = Objects.requireNonNull(endReasonRef, "endReasonRef cannot be null");
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Command just waits - no action needed
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeoutSec);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (!interrupted) {
            // Only set TIMEOUT if we weren't interrupted
            // If interrupted, the other command in the race won
            endReasonRef.set(CommandEndReason.TIMEOUT);
        }
    }
}
