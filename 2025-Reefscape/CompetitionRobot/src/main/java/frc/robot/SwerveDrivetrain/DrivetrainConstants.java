package frc.robot.SwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class DrivetrainConstants {

    // public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.baseUnitMagnitude() * 1.0;
    // public static final double MaxSpeed = 1.6; // meters per second desired top speed
    public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    // public static final double MaxAngularRate = 1.5 * Math.PI; // 1.5 rotations per second max angular velocity
    public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double MaxAcceleration = .4; //2;
}
