package frc.robot.Constants;

/**
 * CAN device IDs for all CAN-based devices on the robot.
 * These include motors, lights, and other CAN devices.
 */
public final class CANIds {
    private CANIds() {
        // Utility class - prevent instantiation
    }

    // Climber
    public static final int CLIMBER = 24;
    public static final int CLIMBER_2 = 31;
    
    // Intake
    public static final int INTAKE_DEPLOY = 25;
    public static final int INTAKE_MOTOR = 26;

    // Floor conveyor
    public static final int FLOOR_CONVEYOR = 27;

    // Feeder
    public static final int FEEDER = 28;

    // Shooter
    public static final int SHOOTER_1 = 29;
    public static final int SHOOTER_2 = 30;
    public static final int SHOOTER_3 = 32;

    // Lights
    public static final int LIGHTS = 40;
}
