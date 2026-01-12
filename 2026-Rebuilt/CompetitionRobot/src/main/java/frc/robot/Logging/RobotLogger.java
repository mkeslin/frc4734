package frc.robot.Logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Wrapper class for AdvantageKit logging.
 * Provides a simplified interface for logging robot data with automatic
 * timestamping and high-frequency recording capabilities.
 * 
 * <p>This class centralizes all logging operations and provides type-safe
 * logging methods for common data types used in FRC robots.
 * 
 * @see Logger
 */
public class RobotLogger {
    private RobotLogger() {
        // Utility class - prevent instantiation
    }

    /**
     * Logs a string message (replaces DataLogManager.log).
     * 
     * @param message The message to log
     */
    public static void log(String message) {
        Logger.recordOutput("Logs/Message", message);
    }

    /**
     * Logs an error message.
     * 
     * @param message The error message to log
     */
    public static void logError(String message) {
        Logger.recordOutput("Logs/Error", message);
    }

    /**
     * Logs a warning message.
     * 
     * @param message The warning message to log
     */
    public static void logWarning(String message) {
        Logger.recordOutput("Logs/Warning", message);
    }

    /**
     * Logs a double value at the specified path.
     * 
     * @param path The logging path (e.g., "Drivetrain/XPosition")
     * @param value The value to log
     */
    public static void recordDouble(String path, double value) {
        Logger.recordOutput(path, value);
    }

    /**
     * Logs a boolean value at the specified path.
     * 
     * @param path The logging path (e.g., "Sensors/CoralInTray")
     * @param value The value to log
     */
    public static void recordBoolean(String path, boolean value) {
        Logger.recordOutput(path, value);
    }

    /**
     * Logs a string value at the specified path.
     * 
     * @param path The logging path (e.g., "State/CurrentState")
     * @param value The value to log
     */
    public static void recordString(String path, String value) {
        Logger.recordOutput(path, value);
    }

    /**
     * Logs a Pose2d (robot position and rotation) at the specified path.
     * 
     * @param path The logging path (e.g., "Drivetrain/Pose")
     * @param pose The pose to log
     */
    public static void recordPose2d(String path, Pose2d pose) {
        Logger.recordOutput(path, pose);
    }

    /**
     * Logs a Rotation2d at the specified path.
     * 
     * @param path The logging path
     * @param rotation The rotation to log
     */
    public static void recordRotation2d(String path, Rotation2d rotation) {
        Logger.recordOutput(path, rotation);
    }

    /**
     * Logs a Translation2d at the specified path.
     * 
     * @param path The logging path
     * @param translation The translation to log
     */
    public static void recordTranslation2d(String path, Translation2d translation) {
        Logger.recordOutput(path, translation);
    }

    /**
     * Logs an array of doubles at the specified path.
     * 
     * @param path The logging path (e.g., "Drivetrain/ModuleStates")
     * @param values The array of values to log
     */
    public static void recordDoubleArray(String path, double[] values) {
        Logger.recordOutput(path, values);
    }


}
