package frc.robot.Constants;

import com.ctre.phoenix6.CANBus;

/**
 * CAN device IDs and bus for all CAN-based devices on the robot.
 * These include motors, lights, and other CAN devices.
 */
public final class CANIds {
    private CANIds() {
        // Utility class - prevent instantiation
    }

    /** CAN bus name for all CTRE devices (e.g. "Canivore" when using a CANivore). Empty string = roboRIO built-in CAN. */
    public static final String CAN_BUS_NAME = "Canivore";

    /** CAN bus instance for TalonFX and other CTRE hardware (preferred over string to avoid deprecation). */
    public static final CANBus CAN_BUS = new CANBus(CAN_BUS_NAME);

    // Climber (lift + jaws)
    public static final int CLIMBER = 40;
    public static final int CLIMBER_JAW_LEFT = 41;
    public static final int CLIMBER_JAW_RIGHT = 42;

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

    // Lights (moved from 40 to avoid conflict with climber)
    public static final int LIGHTS = 50;
}
