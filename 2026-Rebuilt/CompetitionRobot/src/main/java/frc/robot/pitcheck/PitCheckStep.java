package frc.robot.pitcheck;

/**
 * Interface for pit check test steps.
 * 
 * Each step represents a single test that can be run as part of the pit check sequence.
 * Steps are responsible for:
 * 1. Running the test (commanding motors, etc.)
 * 2. Sampling telemetry during execution
 * 3. Evaluating results and returning PASS/WARN/FAIL
 */
public interface PitCheckStep {
    /**
     * Gets the name of this step (for display and identification).
     */
    String getName();

    /**
     * Gets the maximum duration this step should take (seconds).
     * If exceeded, the step will be marked as FAIL with timeout message.
     */
    double getTimeout();

    /**
     * Checks if this step can be run given current safety conditions.
     * 
     * @param robotOnBlocks True if operator confirmed robot is on blocks
     * @param climberMotionAllowed True if operator allowed climber motion
     * @return True if step can run, false if it should be skipped
     */
    boolean canRun(boolean robotOnBlocks, boolean climberMotionAllowed);

    /**
     * Initializes the step before execution.
     * Called once before run() is called.
     * 
     * @return True if initialization succeeded, false to skip this step
     */
    boolean initialize();

    /**
     * Runs the step (commands motors, etc.).
     * This method will be called repeatedly until isComplete() returns true
     * or timeout is reached.
     * 
     * @param elapsedTime Time since step started (seconds)
     */
    void run(double elapsedTime);

    /**
     * Checks if the step execution is complete.
     * 
     * @param elapsedTime Time since step started (seconds)
     * @return True if step should stop running and move to evaluation
     */
    boolean isComplete(double elapsedTime);

    /**
     * Evaluates the step results and returns a result.
     * Called after isComplete() returns true.
     * 
     * @param duration Total duration of the step (seconds)
     * @return Result with PASS/WARN/FAIL status and message
     */
    PitCheckResult evaluate(double duration);

    /**
     * Cleans up after the step (stops motors, etc.).
     * Called after evaluate() or if step is aborted.
     */
    void cleanup();
}
