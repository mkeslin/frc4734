package frc.robot.Auto.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logging.RobotLogger;
import frc.robot.Subsystems.Shooter;
import frc.robot.ShooterSpeeds;

/**
 * Command to spin up the shooter and wait until it reaches target speed.
 *
 * <p>Runs the shooter and finishes when it is within tolerance of the target
 * speed or when the timeout expires. Use in parallel with intake deploy so both
 * complete before shooting.
 *
 * <p>Completion conditions:
 * <ul>
 *   <li>Shooter speed is within rpmTol of target speed</li>
 *   <li>Timeout expires</li>
 * </ul>
 */
public class CmdShooterSpinUpAndWait extends Command {
    private final Shooter shooter;
    private final Supplier<ShooterSpeeds> speedsSupplier;
    private final double rpmTol;
    private final double timeoutSec;
    private Command spinUpCommand;
    private final Timer timer = new Timer();

    /**
     * Creates a new CmdShooterSpinUpAndWait command.
     *
     * @param shooter The shooter subsystem
     * @param speedsSupplier Supplier of per-motor speeds (left, center, right RPS)
     * @param rpmTol RPM tolerance for at-speed check
     * @param timeoutSec Maximum time to wait before finishing
     * @throws NullPointerException if shooter or speedsSupplier is null
     * @throws IllegalArgumentException if timeoutSec is less than or equal to 0
     */
    public CmdShooterSpinUpAndWait(
            Shooter shooter,
            Supplier<ShooterSpeeds> speedsSupplier,
            double rpmTol,
            double timeoutSec) {
        this.shooter = Objects.requireNonNull(shooter, "shooter cannot be null");
        this.speedsSupplier = Objects.requireNonNull(speedsSupplier, "speedsSupplier cannot be null");
        this.rpmTol = rpmTol;
        if (timeoutSec <= 0) {
            throw new IllegalArgumentException("timeoutSec must be greater than 0, got: " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        ShooterSpeeds s = speedsSupplier.get();
        RobotLogger.log(String.format("[CmdShooterSpinUpAndWait] Target speeds: L=%.1f C=%.1f R=%.1f RPS", s.leftRps(), s.centerRps(), s.rightRps()));
        spinUpCommand = shooter.moveToTripleSpeedCommand(speedsSupplier);
        spinUpCommand.initialize();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (spinUpCommand != null) {
            spinUpCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSec)) {
            return true;
        }
        return shooter.isAtSpeed(speedsSupplier.get(), rpmTol);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotLogger.log(String.format("[CmdShooterSpinUpAndWait] t=%.2f Finished: interrupted=%b elapsed=%.2fs",
                Timer.getFPGATimestamp(), interrupted, timer.get()));
        // Shooter keeps spinning; CmdShootForTime will run it during the shot
    }
}
