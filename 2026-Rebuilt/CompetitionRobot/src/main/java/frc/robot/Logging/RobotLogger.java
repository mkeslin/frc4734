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
 * <p>Logging can be controlled via flags in {@link LoggingConfig}.
 * When logging is disabled for a category, methods return immediately without
 * performing any logging operations, minimizing performance impact.
 * 
 * @see Logger
 * @see LoggingConfig
 */
public class RobotLogger {
    private RobotLogger() {
        // Utility class - prevent instantiation
    }

    /**
     * Logs a string message (replaces DataLogManager.log).
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_MESSAGE_LOGGING}.
     * 
     * @param message The message to log
     */
    public static void log(String message) {
        if (!LoggingConfig.ENABLE_MESSAGE_LOGGING) {
            return;
        }
        Logger.recordOutput("Logs/Message", message);
    }

    /**
     * Logs an error message.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_ERROR_LOGGING} or
     * {@link LoggingConfig#ALWAYS_LOG_ERRORS}. If ALWAYS_LOG_ERRORS is true,
     * errors are always logged regardless of ENABLE_ERROR_LOGGING.
     * 
     * @param message The error message to log
     */
    public static void logError(String message) {
        if (!LoggingConfig.ENABLE_ERROR_LOGGING && !LoggingConfig.ALWAYS_LOG_ERRORS) {
            return;
        }
        Logger.recordOutput("Logs/Error", message);
    }

    /**
     * Logs a warning message.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_MESSAGE_LOGGING}.
     * 
     * @param message The warning message to log
     */
    public static void logWarning(String message) {
        if (!LoggingConfig.ENABLE_MESSAGE_LOGGING) {
            return;
        }
        Logger.recordOutput("Logs/Warning", message);
    }

    /**
     * Logs a double value at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path (e.g., "Drivetrain/XPosition")
     * @param value The value to log
     */
    public static void recordDouble(String path, double value) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, value);
    }

    /**
     * Logs a boolean value at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path (e.g., "Sensors/CoralInTray")
     * @param value The value to log
     */
    public static void recordBoolean(String path, boolean value) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, value);
    }

    /**
     * Logs a string value at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path (e.g., "State/CurrentState")
     * @param value The value to log
     */
    public static void recordString(String path, String value) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, value);
    }

    /**
     * Logs a Pose2d (robot position and rotation) at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path (e.g., "Drivetrain/Pose")
     * @param pose The pose to log
     */
    public static void recordPose2d(String path, Pose2d pose) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, pose);
    }

    /**
     * Logs a Rotation2d at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path
     * @param rotation The rotation to log
     */
    public static void recordRotation2d(String path, Rotation2d rotation) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, rotation);
    }

    /**
     * Logs a Translation2d at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path
     * @param translation The translation to log
     */
    public static void recordTranslation2d(String path, Translation2d translation) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, translation);
    }

    /**
     * Logs an array of doubles at the specified path.
     * 
     * <p>Controlled by {@link LoggingConfig#ENABLE_TELEMETRY_LOGGING}.
     * 
     * @param path The logging path (e.g., "Drivetrain/ModuleStates")
     * @param values The array of values to log
     */
    public static void recordDoubleArray(String path, double[] values) {
        if (!LoggingConfig.ENABLE_TELEMETRY_LOGGING) {
            return;
        }
        Logger.recordOutput(path, values);
    }


}
