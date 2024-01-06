// package frc.robot.generated;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

// import edu.wpi.first.math.util.Units;
// import frc.robot.CommandSwerveDrivetrain;

// public class TunerConstants0 {
//         // Both sets of gains need to be tuned to your individual robot
//         // The steer motor uses MotionMagicVoltage control
//         private static final Slot0Configs steerGains = new Slot0Configs()
//                         .withKP(100).withKI(0).withKD(0.05)
//                         .withKS(0).withKV(1.5).withKA(0);
//         // When using closed-loop control, the drive motor uses:
//         // - VelocityVoltage, if DrivetrainConstants.SupportsPro is false (default)
//         // - VelocityTorqueCurrentFOC, if DrivetrainConstants.SupportsPro is true
//         private static final Slot0Configs driveGains = new Slot0Configs()
//                         .withKP(3).withKI(0).withKD(0)
//                         .withKS(0).withKV(0).withKA(0);

//         // The stator current at which the wheels start to slip;
//         // This needs to be tuned to your individual robot
//         private static final double kSlipCurrentA = 300.0;

//         // Theoretical free speed (m/s) at 12v applied output;
//         // This needs to be tuned to your individual robot
//         private static final double kSpeedAt12VoltsMps = 6.0;

//         // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
//         // This may need to be tuned to your individual robot
//         private static final double kCoupleRatio = 0;

//         // private static final double kDriveGearRatio = 1; // todo 0;
//         // private static final double kSteerGearRatio = 15.42857142857143;
//         // private static final double kWheelRadiusInches = 2; // todo

//         private static final double kDriveGearRatio = 5.5;      // 22 - 26
//         private static final double kSteerGearRatio = 5.5;
//         private static final double kWheelRadiusInches = 2;

//         private static final boolean kSteerMotorReversed = false;
//         private static final boolean kInvertLeftSide = false;
//         private static final boolean kInvertRightSide = true;

//         private static final String kCANbusName = "";
//         private static final int kPigeonId = 0;

//         // These are only used for simulation
//         private static final double kSteerInertia = 0.00001;
//         private static final double kDriveInertia = 0.001;

//         private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
//                         .withPigeon2Id(kPigeonId)
//                         .withCANbusName(kCANbusName);

//         private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
//                         .withDriveMotorGearRatio(kDriveGearRatio)
//                         .withSteerMotorGearRatio(kSteerGearRatio)
//                         .withWheelRadius(kWheelRadiusInches)
//                         .withSlipCurrent(kSlipCurrentA)
//                         .withSteerMotorGains(steerGains)
//                         .withDriveMotorGains(driveGains)
//                         .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
//                         .withSteerInertia(kSteerInertia)
//                         .withDriveInertia(kDriveInertia)
//                         .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
//                         .withCouplingGearRatio(kCoupleRatio)
//                         .withSteerMotorInverted(kSteerMotorReversed);

//         // Front Left
//         private static final int kFrontLeftDriveMotorId = 4;
//         private static final int kFrontLeftSteerMotorId = 6;
//         private static final int kFrontLeftEncoderId = 1;
//         private static final double kFrontLeftEncoderOffset = -0.004150390625;

//         private static final double kFrontLeftXPosInches = 0;
//         private static final double kFrontLeftYPosInches = 0;

//         // Front Right
//         private static final int kFrontRightDriveMotorId = 2;
//         private static final int kFrontRightSteerMotorId = 0;
//         private static final int kFrontRightEncoderId = 0;
//         private static final double kFrontRightEncoderOffset = 0.44384765625;

//         private static final double kFrontRightXPosInches = 0;
//         private static final double kFrontRightYPosInches = -0;

//         // Back Left
//         private static final int kBackLeftDriveMotorId = 3;
//         private static final int kBackLeftSteerMotorId = 7;
//         private static final int kBackLeftEncoderId = 2;
//         private static final double kBackLeftEncoderOffset = -0.143798828125;

//         private static final double kBackLeftXPosInches = -0;
//         private static final double kBackLeftYPosInches = 0;

//         // Back Right
//         private static final int kBackRightDriveMotorId = 5;
//         private static final int kBackRightSteerMotorId = 1;
//         private static final int kBackRightEncoderId = 3;
//         private static final double kBackRightEncoderOffset = 0.47412109375;

//         private static final double kBackRightXPosInches = -0;
//         private static final double kBackRightYPosInches = -0;

//         private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
//                         kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
//                         Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
//                         kInvertLeftSide);
//         private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
//                         kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
//                         kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
//                         Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
//         private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
//                         kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
//                         Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
//                         kInvertLeftSide);
//         private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
//                         kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
//                         Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
//                         kInvertRightSide);

//         public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
//                         FrontLeft,
//                         FrontRight, BackLeft, BackRight);
// }
