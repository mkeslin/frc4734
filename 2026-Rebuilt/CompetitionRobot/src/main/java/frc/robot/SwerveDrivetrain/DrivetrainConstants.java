package frc.robot.SwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class DrivetrainConstants {

    // public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.baseUnitMagnitude() * 1.0;
    // public static final double MaxSpeed = 1.6; // meters per second desired top speed
    public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.in(MetersPerSecond) * .7; // kSpeedAt12Volts desired top speed

    // public static final double MaxAngularRate = 1.5 * Math.PI; // 1.5 rotations per second max angular velocity
    public static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity (2π rad/s)

    public static final double MaxAcceleration = .4; //2;

    // Rate limiting constants for joystick inputs (-1..1 range)
    // These are baseline defaults and can be tuned for your robot
    // Units are in 1/s (per second), applied to normalized joystick inputs before scaling to physical units
    public static final double kTranslationSlewRate = 2.5; // 1/s for X and Y translation inputs
    public static final double kRotationSlewRate = 4.0; // 1/s for rotation (omega) input

    // Joystick input processing constants
    public static final double kJoystickDeadband = 0.1; // Deadband threshold for joystick inputs (-1..1 range)

    // Turtle mode constants
    public static final double kTurtleSpeedMultiplier = 0.15; // Speed multiplier for turtle mode (15% of max speed)
    public static final double kTurtleAngularRate = Math.PI * 0.5; // Max angular velocity for turtle mode (rad/s)
}
