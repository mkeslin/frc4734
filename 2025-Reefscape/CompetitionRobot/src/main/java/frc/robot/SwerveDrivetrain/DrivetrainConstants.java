package frc.robot.SwerveDrivetrain;

public class DrivetrainConstants {

    public static final double MaxSpeed = SwerveDrivetrainA.kSpeedAt12Volts.baseUnitMagnitude();
    // public static final double MaxSpeed = 1.6; // meters per second desired top speed
    public static final double MaxAngularRate = 1.5 * Math.PI; // 1.5 rotations per second max angular velocity
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double MaxAcceleration = .4; //2;
}
