package frc.robot.pitcheck;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Pit Check Runner - State machine for pre-match hardware verification.
 * 
 * <p><b>Expected Runtime:</b> Typically 30-45 seconds for full check.
 * 
 * <p><b>Safety Gates:</b>
 * <ul>
 *   <li>Will NOT run if FMS is attached (DriverStation.isFMSAttached())</li>
 *   <li>Drivetrain tests require "RobotOnBlocks" toggle to be true</li>
 *   <li>Climber motion tests require "ClimberMotionAllowed" toggle to be true</li>
 *   <li>Can be aborted at any time via Shuffleboard button</li>
 * </ul>
 * 
 * <p><b>Result Interpretation:</b>
 * <ul>
 *   <li><b>PASS:</b> All checks passed, robot ready for match</li>
 *   <li><b>WARN:</b> Some values marginal but acceptable (e.g., current slightly low)</li>
 *   <li><b>FAIL:</b> Critical issue detected (motor unplugged, encoder not moving, etc.)</li>
 * </ul>
 * 
 * <p><b>Typical Workflow:</b>
 * <ol>
 *   <li>Place robot on blocks</li>
 *   <li>Set "RobotOnBlocks" toggle to true</li>
 *   <li>Press "Run Pit Check"</li>
 *   <li>Review results on Shuffleboard</li>
 *   <li>Fix any FAIL items</li>
 *   <li>Rerun targeted steps or full check</li>
 * </ol>
 */
public class PitCheckRunner extends Command {
    public enum State {
        IDLE,
        PRECHECK_GATES,
        RUNNING_STEP,
        STEP_EVALUATION,
        COMPLETE,
        ABORTED
    }

    private final List<PitCheckStep> steps;
    private final PitCheckShuffleboard shuffleboard;
    private final Runnable stopAllSubsystems;
    
    private State currentState;
    private PitCheckReport report;
    private PitCheckStep currentStep;
    private int currentStepIndex;
    private double stepStartTime;
    private PitCheckResult lastResult;
    private boolean wasRunning;

    /**
     * Creates a new pit check runner.
     * 
     * @param steps List of test steps to run (in order)
     * @param shuffleboard Shuffleboard UI manager
     * @param stopAllSubsystems Runnable that stops all subsystems (called on abort)
     */
    public PitCheckRunner(
            List<PitCheckStep> steps,
            PitCheckShuffleboard shuffleboard,
            Runnable stopAllSubsystems) {
        this.steps = new ArrayList<>(Objects.requireNonNull(steps, "steps cannot be null"));
        this.shuffleboard = Objects.requireNonNull(shuffleboard, "shuffleboard cannot be null");
        this.stopAllSubsystems = Objects.requireNonNull(stopAllSubsystems, "stopAllSubsystems cannot be null");
        
        this.currentState = State.IDLE;
        this.report = null;
        this.currentStep = null;
        this.currentStepIndex = -1;
        this.stepStartTime = 0.0;
        this.lastResult = null;
        this.wasRunning = false;

        // Add step names to chooser
        for (PitCheckStep step : steps) {
            shuffleboard.addStepToChooser(step.getName());
        }
    }

    @Override
    public void initialize() {
        // Check FMS gate
        if (DriverStation.isFMSAttached()) {
            currentState = State.ABORTED;
            if (report != null) {
                report.abort("FMS attached - pit check disabled");
            }
            return;
        }

        // Initialize new report
        report = new PitCheckReport();
        report.setRunning();
        currentState = State.PRECHECK_GATES;
        currentStepIndex = -1;
        lastResult = null;
        wasRunning = true;
    }

    @Override
    public void execute() {
        // Check for abort
        if (shuffleboard.getAbortPitCheck()) {
            abort("Operator aborted");
            shuffleboard.resetControls();
            return;
        }

        // Check FMS gate (recheck periodically)
        if (DriverStation.isFMSAttached()) {
            abort("FMS attached during execution");
            return;
        }

        switch (currentState) {
            case IDLE:
                // Waiting for run command
                break;

            case PRECHECK_GATES:
                // Check all gates before starting
                if (!checkGates()) {
                    abort("Safety gates not met");
                    return;
                }
                currentState = State.RUNNING_STEP;
                currentStepIndex = 0;
                break;

            case RUNNING_STEP:
                runCurrentStep();
                break;

            case STEP_EVALUATION:
                evaluateCurrentStep();
                break;

            case COMPLETE:
            case ABORTED:
                // Terminal states - do nothing
                break;
        }

        // Update UI
        String currentStepName = currentStep != null ? currentStep.getName() : null;
        shuffleboard.update(currentState, currentStepName, lastResult, report);
    }

    private boolean checkGates() {
        // FMS check already done in initialize()
        // Additional gates can be added here
        return true;
    }

    private void runCurrentStep() {
        // Find next runnable step
        while (currentStepIndex < steps.size()) {
            if (currentStepIndex < 0) {
                currentStepIndex = 0;
            }

            PitCheckStep step = steps.get(currentStepIndex);
            boolean robotOnBlocks = shuffleboard.getRobotOnBlocks();
            boolean climberMotionAllowed = shuffleboard.getClimberMotionAllowed();

            // Check if step can run
            if (!step.canRun(robotOnBlocks, climberMotionAllowed)) {
                // Skip this step
                currentStepIndex++;
                if (currentStepIndex >= steps.size()) {
                    // All steps done
                    complete();
                    return;
                }
                continue;
            }

            // Initialize step
            if (currentStep == null) {
                if (!step.initialize()) {
                    // Step initialization failed, skip it
                    lastResult = new PitCheckResult(
                        step.getName(),
                        PitCheckResult.Status.FAIL,
                        "Initialization failed",
                        0.0
                    );
                    report.addResult(lastResult);
                    currentStepIndex++;
                    continue;
                }
                currentStep = step;
                stepStartTime = Timer.getFPGATimestamp();
            }

            // Run step
            double elapsedTime = Timer.getFPGATimestamp() - stepStartTime;
            step.run(elapsedTime);

            // Check timeout
            if (elapsedTime > step.getTimeout()) {
                // Step timed out
                step.cleanup();
                lastResult = new PitCheckResult(
                    step.getName(),
                    PitCheckResult.Status.FAIL,
                    String.format("Timeout after %.2fs", elapsedTime),
                    elapsedTime
                );
                report.addResult(lastResult);
                currentStep = null;
                currentStepIndex++;
                currentState = State.STEP_EVALUATION;
                return;
            }

            // Check if step is complete
            if (step.isComplete(elapsedTime)) {
                currentState = State.STEP_EVALUATION;
                return;
            }

            // Step still running
            return;
        }

        // All steps done
        complete();
    }

    private void evaluateCurrentStep() {
        if (currentStep == null) {
            currentState = State.RUNNING_STEP;
            return;
        }

        double duration = Timer.getFPGATimestamp() - stepStartTime;
        PitCheckResult result = currentStep.evaluate(duration);
        report.addResult(result);
        lastResult = result;

        currentStep.cleanup();
        currentStep = null;
        currentStepIndex++;

        // Check if all steps done
        if (currentStepIndex >= steps.size()) {
            complete();
        } else {
            // Move to next step
            Timer.delay(PitCheckConstants.STEP_DELAY);
            currentState = State.RUNNING_STEP;
        }
    }

    private void complete() {
        currentState = State.COMPLETE;
        report.complete();
        wasRunning = false;
    }

    private void abort(String reason) {
        currentState = State.ABORTED;
        
        // Stop current step
        if (currentStep != null) {
            currentStep.cleanup();
            currentStep = null;
        }

        // Stop all subsystems
        stopAllSubsystems.run();

        // Mark report as aborted
        if (report != null) {
            report.abort(reason);
        }

        wasRunning = false;
    }

    /**
     * Runs a single selected step (for rerunning individual tests).
     */
    public void runSelectedStep() {
        String stepName = shuffleboard.getSelectedStep();
        if (stepName == null || stepName.isEmpty()) {
            return;
        }

        // Find the step
        PitCheckStep selectedStep = null;
        for (PitCheckStep step : steps) {
            if (step.getName().equals(stepName)) {
                selectedStep = step;
                break;
            }
        }

        if (selectedStep == null) {
            return;
        }

        // Check gates
        if (DriverStation.isFMSAttached()) {
            return;
        }

        boolean robotOnBlocks = shuffleboard.getRobotOnBlocks();
        boolean climberMotionAllowed = shuffleboard.getClimberMotionAllowed();

        if (!selectedStep.canRun(robotOnBlocks, climberMotionAllowed)) {
            return;
        }

        // Run the step
        if (!selectedStep.initialize()) {
            return;
        }

        double startTime = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - startTime < selectedStep.getTimeout()) {
            double elapsed = Timer.getFPGATimestamp() - startTime;
            selectedStep.run(elapsed);
            
            if (selectedStep.isComplete(elapsed)) {
                break;
            }
            
            Timer.delay(0.02); // 20ms loop
        }

        double duration = Timer.getFPGATimestamp() - startTime;
        PitCheckResult result = selectedStep.evaluate(duration);
        selectedStep.cleanup();

        // Update UI with result
        lastResult = result;
        shuffleboard.update(State.COMPLETE, stepName, result, null);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && wasRunning) {
            abort("Command interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == State.COMPLETE || currentState == State.ABORTED;
    }

    // Getters
    public State getCurrentState() {
        return currentState;
    }

    public PitCheckReport getReport() {
        return report;
    }

    public boolean isRunning() {
        return currentState == State.RUNNING_STEP || 
               currentState == State.STEP_EVALUATION ||
               currentState == State.PRECHECK_GATES;
    }
}
