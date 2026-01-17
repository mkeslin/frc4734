package frc.robot.SwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class DrivetrainConstants {

    // public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.baseUnitMagnitude() * 1.0;
    // public static final double MaxSpeed = 1.6; // meters per second desired top speed
    public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.in(MetersPerSecond) * .7; // kSpeedAt12Volts desired top speed

    // public static final double MaxAngularRate = 1.5 * Math.PI; // 1.5 rotations per second max angular velocity
    public static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity (2π rad/s)

    public static final double MaxAcceleration = 2.0; // m/s² - tuned for swerve drive capabilities

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

    // PathPlanner PID constants
    // These control how aggressively the robot follows paths
    // Translation: controls X and Y position error correction
    // Rotation: controls heading error correction
    public static final PIDConstants kPathTranslationPID = new PIDConstants(7, 0, 0);
    public static final PIDConstants kPathRotationPID = new PIDConstants(7, 0, 0);

    // Pose estimator standard deviations
    // Used for Kalman filtering in pose estimation
    // Units: [x, y, theta] in meters, meters, radians
    // Odometry: lower values = more trust in wheel encoder measurements
    // Vision: higher values = less trust in vision measurements (vision is noisier)
    public static final Matrix<N3, N1> kOdometryStandardDeviation = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> kVisionStandardDeviation = VecBuilder.fill(0.5, 0.5, 0.5);

    // PathPlanner angular acceleration constraint
    // Maximum angular acceleration for path following (rad/s²)
    // Calculated as reasonable default: ~540°/s² = ~9.4 rad/s²
    public static final double kMaxAngularAcceleration = Math.toRadians(540); // rad/s²

    // Static initializer to validate constants at startup
    static {
        assert MaxSpeed > 0 && MaxSpeed <= 10 : "MaxSpeed out of reasonable range (0-10 m/s)";
        assert MaxAngularRate > 0 && MaxAngularRate <= 10 : "MaxAngularRate out of reasonable range (0-10 rad/s)";
        assert MaxAcceleration > 0 && MaxAcceleration <= 5 : "MaxAcceleration out of reasonable range (0-5 m/s²)";
    }
}
