package frc.robot.autotest;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Command wrapper that logs telemetry at start and end, and determines end reason.
 * 
 * <p>This wrapper captures subsystem state before and after command execution,
 * determines why the command ended (SUCCESS, TIMEOUT, INTERRUPTED, etc.), and
 * publishes results to SmartDashboard/NetworkTables.
 * 
 * <p>Uses deterministic timeout detection via raceWith() pattern with TimeoutCommand.
 */
public class LoggedCommand extends Command {
    private final String testName;
    private final Command innerCommand;
    private final double timeoutSec;
    private final Supplier<Rotation2d> headingTargetOpt;
    
    // Subsystems for telemetry
    private final CommandSwerveDrivetrain drive;
    private final PhotonVision vision;
    private final Shooter shooter;
    private final Feeder feeder;
    private final DeployableIntake intake;
    private final Climber climber;
    private final PositionTracker positionTracker;
    
    // Runtime state
    private final Timer timer = new Timer();
    private final AtomicReference<CommandEndReason> endReasonRef = new AtomicReference<>(CommandEndReason.SUCCESS);
    private Command wrappedCommand;
    private Map<String, Double> metricsStart;
    private List<CommandRunResult> resultHistory;
    private int maxHistorySize;

    /**
     * Creates a new LoggedCommand.
     * 
     * @param testName Name of the test (for logging)
     * @param innerCommand The command to wrap and log
     * @param timeoutSec Timeout in seconds (0 = no timeout)
     * @param headingTargetOpt Optional target heading for error calculation
     * @param drive Drivetrain subsystem (required)
     * @param vision Vision subsystem (optional)
     * @param shooter Shooter subsystem (optional)
     * @param feeder Feeder subsystem (optional)
     * @param intake Intake subsystem (optional)
     * @param climber Climber subsystem (optional)
     * @param positionTracker Position tracker (optional)
     * @param resultHistory List to add results to (optional)
     * @param maxHistorySize Maximum history size (defaults to 20)
     * @throws NullPointerException if testName, innerCommand, or drive is null
     */
    public LoggedCommand(
            String testName,
            Command innerCommand,
            double timeoutSec,
            Supplier<Rotation2d> headingTargetOpt,
            CommandSwerveDrivetrain drive,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker,
            List<CommandRunResult> resultHistory,
            int maxHistorySize) {
        this.testName = Objects.requireNonNull(testName, "testName cannot be null");
        this.innerCommand = Objects.requireNonNull(innerCommand, "innerCommand cannot be null");
        this.timeoutSec = timeoutSec;
        this.headingTargetOpt = headingTargetOpt;
        this.drive = Objects.requireNonNull(drive, "drive cannot be null");
        this.vision = vision;
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.climber = climber;
        this.positionTracker = positionTracker;
        this.resultHistory = resultHistory;
        this.maxHistorySize = maxHistorySize > 0 ? maxHistorySize : 20;
        
        // Add requirements from inner command
        addRequirements(innerCommand.getRequirements().toArray(new edu.wpi.first.wpilibj2.command.Subsystem[0]));
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        endReasonRef.set(CommandEndReason.SUCCESS);
        
        // Capture start telemetry
        Optional<Rotation2d> headingTarget = headingTargetOpt != null ? Optional.ofNullable(headingTargetOpt.get()) : Optional.empty();
        metricsStart = TelemetrySnapshot.capture(
                drive, vision, shooter, feeder, intake, climber, positionTracker, headingTarget);
        
        // Wrap with timeout if specified
        if (timeoutSec > 0) {
            Command timeoutCmd = new TimeoutCommand(timeoutSec, endReasonRef);
            wrappedCommand = innerCommand.raceWith(timeoutCmd);
        } else {
            wrappedCommand = innerCommand;
        }
        
        wrappedCommand.initialize();
    }

    @Override
    public void execute() {
        if (wrappedCommand != null) {
            wrappedCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (wrappedCommand != null) {
            return wrappedCommand.isFinished();
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        double duration = timer.get();
        
        if (wrappedCommand != null) {
            wrappedCommand.end(interrupted);
        }
        
        // Capture end telemetry
        Optional<Rotation2d> headingTarget = headingTargetOpt != null ? Optional.ofNullable(headingTargetOpt.get()) : Optional.empty();
        Map<String, Double> metricsEnd = TelemetrySnapshot.capture(
                drive, vision, shooter, feeder, intake, climber, positionTracker, headingTarget);
        
        // Determine end reason
        CommandEndReason reason;
        if (interrupted) {
            reason = CommandEndReason.INTERRUPTED;
        } else if (endReasonRef.get() == CommandEndReason.TIMEOUT) {
            reason = CommandEndReason.TIMEOUT;
        } else if (wrappedCommand != null && wrappedCommand.isFinished()) {
            reason = CommandEndReason.SUCCESS;
        } else {
            reason = CommandEndReason.ERROR;
        }
        
        // Create result
        CommandRunResult result = CommandRunResult.create(
                testName, reason, duration, metricsStart, metricsEnd);
        
        // Publish to SmartDashboard
        SmartDashboard.putString("AutoTest/LastResult/Name", result.name());
        SmartDashboard.putString("AutoTest/LastResult/Reason", result.reason().toString());
        SmartDashboard.putNumber("AutoTest/LastResult/Duration", result.durationSec());
        
        // Add to history
        if (resultHistory != null) {
            resultHistory.add(result);
            // Trim to max size
            while (resultHistory.size() > maxHistorySize) {
                resultHistory.remove(0);
            }
        }
    }

    /**
     * Static helper to create a LoggedCommand with default settings.
     * 
     * @param name Test name
     * @param innerCommand Command to wrap
     * @param timeoutSec Timeout (0 = no timeout)
     * @param headingTargetOpt Optional heading target
     * @param drive Drivetrain
     * @param vision Vision (can be null)
     * @param shooter Shooter (can be null)
     * @param feeder Feeder (can be null)
     * @param intake Intake (can be null)
     * @param climber Climber (can be null)
     * @param positionTracker Position tracker (can be null)
     * @param resultHistory Result history list (can be null)
     * @return LoggedCommand instance
     */
    public static Command logWrap(
            String name,
            Command innerCommand,
            double timeoutSec,
            Supplier<Rotation2d> headingTargetOpt,
            CommandSwerveDrivetrain drive,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker,
            List<CommandRunResult> resultHistory) {
        return new LoggedCommand(
                name, innerCommand, timeoutSec, headingTargetOpt,
                drive, vision, shooter, feeder, intake, climber, positionTracker,
                resultHistory, 20);
    }
}
