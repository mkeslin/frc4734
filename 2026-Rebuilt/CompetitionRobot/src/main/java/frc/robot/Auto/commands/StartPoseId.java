package frc.robot.Auto.commands;

/**
 * Enumeration of autonomous starting positions.
 * Used for selecting initial robot pose from a predefined map.
 */
public enum StartPoseId {
    /**
     * Starting position 1 (typically left side of field).
     */
    POS_1,

    /**
     * Starting position 2 (typically center of field).
     */
    POS_2,

    /**
     * Starting position 3 (typically right side of field).
     */
    POS_3,

    /**
     * Test - Climb start: 4 ft from tower center toward field center and 4 ft toward closest sideline.
     * Used only by Test - Climb routine; pose depends on selected climb side.
     */
    POS_TEST_CLIMB
}
