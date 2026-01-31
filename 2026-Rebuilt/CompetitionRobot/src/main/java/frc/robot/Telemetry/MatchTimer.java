package frc.robot.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Logging.RobotLogger;

/**
 * Utility class for tracking match time and phases.
 *
 * <p>Provides methods to:
 * <ul>
 *   <li>Get current match time and time remaining
 *   <li>Detect current match phase (AUTO, TELEOP, ENDGAME, DISABLED)
 *   <li>Calculate time remaining for each phase
 *   <li>Log phase changes to AdvantageKit
 * </ul>
 */
public final class MatchTimer {
    /** Autonomous period duration in seconds */
    public static final double AUTO_DURATION = 15.0;

    /** Teleop period duration in seconds */
    public static final double TELEOP_DURATION = 135.0;

    /** Endgame start time (seconds remaining in teleop) */
    public static final double ENDGAME_START = 30.0;

    /** Total match duration (auto + teleop) */
    public static final double TOTAL_MATCH_DURATION = AUTO_DURATION + TELEOP_DURATION;

    private static MatchPhase lastLoggedPhase = MatchPhase.DISABLED;

    private MatchTimer() {
        // Utility class - prevent instantiation
    }

    /** Enumeration of match phases. */
    public enum MatchPhase {
        DISABLED,
        AUTO,
        TELEOP,
        ENDGAME
    }

    /**
     * Gets the current match time from DriverStation.
     *
     * @return Match time in seconds, or -1 if match not started or disabled
     */
    public static double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    /**
     * Gets the current match phase.
     *
     * @return Current phase (DISABLED, AUTO, TELEOP, or ENDGAME)
     */
    public static MatchPhase getPhase() {
        if (!DriverStation.isEnabled()) {
            return MatchPhase.DISABLED;
        }

        double matchTime = getMatchTime();
        if (matchTime < 0) {
            return MatchPhase.DISABLED;
        }

        if (matchTime < AUTO_DURATION) {
            return MatchPhase.AUTO;
        }

        double teleopTime = matchTime - AUTO_DURATION;
        double teleopRemaining = TELEOP_DURATION - teleopTime;

        if (teleopRemaining <= ENDGAME_START) {
            return MatchPhase.ENDGAME;
        }

        return MatchPhase.TELEOP;
    }

    /** Returns true if the robot is currently in autonomous period. */
    public static boolean isAutoTime() {
        return getPhase() == MatchPhase.AUTO;
    }

    /** Returns true if the robot is currently in teleop period (excluding endgame). */
    public static boolean isTeleopTime() {
        return getPhase() == MatchPhase.TELEOP;
    }

    /** Returns true if the robot is currently in endgame (last 30 seconds). */
    public static boolean isEndgame() {
        return getPhase() == MatchPhase.ENDGAME;
    }

    /** Gets the time remaining in autonomous period in seconds. */
    public static double getAutoTimeRemaining() {
        if (!isAutoTime()) {
            return 0.0;
        }
        return Math.max(0.0, AUTO_DURATION - getMatchTime());
    }

    /** Gets the time remaining in teleop period in seconds (including endgame). */
    public static double getTeleopTimeRemaining() {
        double matchTime = getMatchTime();
        if (matchTime < AUTO_DURATION) {
            return TELEOP_DURATION;
        }
        double teleopTime = matchTime - AUTO_DURATION;
        return Math.max(0.0, TELEOP_DURATION - teleopTime);
    }

    /** Gets the time remaining in endgame in seconds. */
    public static double getEndgameTimeRemaining() {
        if (!isEndgame()) {
            return 0.0;
        }
        return getTeleopTimeRemaining();
    }

    /**
     * Logs phase changes to AdvantageKit. Call periodically (e.g., in robotPeriodic).
     */
    public static void logPhaseChanges() {
        MatchPhase currentPhase = getPhase();
        if (currentPhase != lastLoggedPhase) {
            RobotLogger.log(
                    String.format("[MatchTimer] Phase changed: %s -> %s", lastLoggedPhase, currentPhase));
            RobotLogger.recordString("MatchTimer/Phase", currentPhase.toString());
            RobotLogger.recordDouble("MatchTimer/MatchTime", getMatchTime());
            lastLoggedPhase = currentPhase;
        }
    }

    /**
     * Formats match time as MM:SS.mmm for display.
     *
     * @param timeSeconds Time in seconds
     * @return Formatted string (e.g., "2:15.300")
     */
    public static String formatTime(double timeSeconds) {
        if (timeSeconds < 0) {
            return "--:--";
        }
        int minutes = (int) (timeSeconds / 60.0);
        int seconds = (int) (timeSeconds % 60.0);
        int milliseconds = (int) ((timeSeconds % 1.0) * 1000.0);
        return String.format("%d:%02d.%03d", minutes, seconds, milliseconds);
    }

    /**
     * Formats time remaining as MM:SS for display.
     *
     * @param timeSeconds Time remaining in seconds
     * @return Formatted string (e.g., "2:15")
     */
    public static String formatTimeRemaining(double timeSeconds) {
        if (timeSeconds < 0) {
            return "--:--";
        }
        int minutes = (int) (timeSeconds / 60.0);
        int seconds = (int) (timeSeconds % 60.0);
        return String.format("%d:%02d", minutes, seconds);
    }
}
