package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Centralized telemetry class for NetworkTables usage.
 * Provides shared NetworkTable instances and publisher creation methods
 * to avoid scattered telemetry across multiple subsystem files.
 */
public class TelemetryCalcs {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    
    // Shared NetworkTable instances for different telemetry categories
    private static final NetworkTable MECHANISMS_TABLE = INSTANCE.getTable("Mechanisms");
    private static final NetworkTable POSE_TABLE = INSTANCE.getTable("Pose");
    private static final NetworkTable DRIVE_TABLE = INSTANCE.getTable("Drive");

    private TelemetryCalcs() {
        // Utility class - prevent instantiation
    }

    /**
     * Creates a DoublePublisher for the Mechanisms table.
     * 
     * @param topic The topic name (e.g., "Elevator Position", "Arm Angle")
     * @return A DoublePublisher for the specified topic
     */
    public static DoublePublisher createMechanismsPublisher(String topic) {
        return MECHANISMS_TABLE.getDoubleTopic(topic).publish();
    }

    /**
     * Gets the Mechanisms NetworkTable instance.
     * 
     * @return The Mechanisms NetworkTable
     */
    public static NetworkTable getMechanismsTable() {
        return MECHANISMS_TABLE;
    }

    /**
     * Gets the Pose NetworkTable instance.
     * 
     * @return The Pose NetworkTable
     */
    public static NetworkTable getPoseTable() {
        return POSE_TABLE;
    }

    /**
     * Gets the Drive NetworkTable instance.
     * 
     * @return The Drive NetworkTable
     */
    public static NetworkTable getDriveTable() {
        return DRIVE_TABLE;
    }

    /**
     * Gets the default NetworkTableInstance.
     * 
     * @return The default NetworkTableInstance
     */
    public static NetworkTableInstance getInstance() {
        return INSTANCE;
    }
}
