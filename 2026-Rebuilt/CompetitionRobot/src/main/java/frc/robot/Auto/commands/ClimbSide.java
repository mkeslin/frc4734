package frc.robot.Auto.commands;

/**
 * Which side of the tower bar to align to for climbing.
 * Center is not physically possible; use Shuffleboard to select Left or Right.
 */
public enum ClimbSide {
    /** Climb on the left side of the bar (high Y in blue alliance). */
    LEFT,

    /** Climb on the right side of the bar (low Y in blue alliance). */
    RIGHT
}
