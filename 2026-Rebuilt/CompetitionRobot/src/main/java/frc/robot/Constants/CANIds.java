package frc.robot.Constants;

/**
 * CAN device IDs for all CAN-based devices on the robot.
 * These include motors, lights, and other CAN devices.
 */
public final class CANIds {
    private CANIds() {
        // Utility class - prevent instantiation
    }

    // Elevator
    public static final int ELEVATOR_LEFT = 20;
    public static final int ELEVATOR_RIGHT = 21;

    // Mechanisms
    public static final int SIDE_TO_SIDE = 22;
    public static final int ARM = 23;
    public static final int CLIMBER = 24;
    
    // Intake
    public static final int INTAKE_DEPLOY = 25;
    public static final int INTAKE_MOTOR = 26;

    // Floor conveyor
    public static final int FLOOR_CONVEYOR = 27;

    // Feeder
    public static final int FEEDER = 28;

    // Lights
    public static final int LIGHTS = 40;
}
