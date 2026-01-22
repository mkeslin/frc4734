package frc.robot.autotest;

/**
 * Enumeration of reasons why a command ended.
 * Used for test result tracking and debugging.
 */
public enum CommandEndReason {
    /**
     * Command completed successfully (finished normally).
     */
    SUCCESS,

    /**
     * Command timed out (timeout command won the race).
     */
    TIMEOUT,

    /**
     * Command was interrupted (canceled externally).
     */
    INTERRUPTED,

    /**
     * Command ended due to a condition being false (e.g., vision quality too poor).
     */
    CONDITION_FALSE,

    /**
     * Command ended due to an error or unexpected condition.
     */
    ERROR
}
