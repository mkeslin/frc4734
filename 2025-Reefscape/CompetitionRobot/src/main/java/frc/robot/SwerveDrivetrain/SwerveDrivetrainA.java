package frc.robot.SwerveDrivetrain;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.swerve.SwerveModule.ClosedLoopOutputType;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;

public class SwerveDrivetrainA {

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100)
            .withKI(0)
            // .withKD(0.2)
            .withKD(0.05)
            .withKS(0)
            .withKV(1.5)
            .withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(3)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput =
            // ClosedLoopOutputType.TorqueCurrentFOC;
            ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput =
            // ClosedLoopOutputType.TorqueCurrentFOC;
            ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    // private static final double kCoupleRatio = 2;
    private static final double kCoupleRatio = 3.5;

    private static final double kDriveGearRatio = 5.5;
    private static final double kSteerGearRatio = 10.285714285714286;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    // private static final boolean kInvertLeftSide = false;
    // private static final boolean kInvertRightSide = false;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 0;
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these
    // cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            // Swerve azimuth does not require much torque output, so we can set a
                            // relatively low
                            // stator current limit to help avoid brownouts without impacting performance.
                            .withStatorCurrentLimit(Amps.of(60))
                            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    // private static final SwerveDrivetrainConstants DrivetrainConstants = new
    // SwerveDrivetrainConstants()
    // .withPigeon2Id(kPigeonId)
    // .withCANbusName(kCANbusName);
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANbusName)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    // private static final SwerveModuleConstantsFactory ConstantCreator = new
    // SwerveModuleConstantsFactory()
    // .withDriveMotorGearRatio(kDriveGearRatio)
    // .withSteerMotorGearRatio(kSteerGearRatio)
    // .withWheelRadius(kWheelRadiusInches)
    // .withSlipCurrent(kSlipCurrentA)
    // .withSteerMotorGains(steerGains)
    // .withDriveMotorGains(driveGains)
    // .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
    // .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
    // .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
    // .withSteerInertia(kSteerInertia)
    // .withDriveInertia(kDriveInertia)
    // .withSteerFrictionVoltage(kSteerFrictionVoltage)
    // .withDriveFrictionVoltage(kDriveFrictionVoltage)
    // .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
    // .withCouplingGearRatio(kCoupleRatio)
    // .withSteerMotorInverted(kSteerMotorReversed);
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12VoltsMps)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 4;
    private static final int kFrontLeftSteerMotorId = 6;
    private static final int kFrontLeftEncoderId = 1;
    private static final double kFrontLeftEncoderOffset = 0.496826171875;

    private static final double kFrontLeftXPosInches = 10.5;
    private static final double kFrontLeftYPosInches = 10.5;

    // Front Right
    private static final int kFrontRightDriveMotorId = 2;
    private static final int kFrontRightSteerMotorId = 13;
    private static final int kFrontRightEncoderId = 0;
    // private static final double kFrontRightEncoderOffset = 0.19140625;
    private static final double kFrontRightEncoderOffset = 0.1982421875;

    private static final double kFrontRightXPosInches = 10.5;
    private static final double kFrontRightYPosInches = -10.5;

    // Back Left
    private static final int kBackLeftDriveMotorId = 3;
    private static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 2;
    // private static final double kBackLeftEncoderOffset = -0.143798828125;
    private static final double kBackLeftEncoderOffset = 0.113798828125;

    private static final double kBackLeftXPosInches = -10.5;
    private static final double kBackLeftYPosInches = 10.5;

    // Back Right
    private static final int kBackRightDriveMotorId = 5;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 3;
    // private static final double kBackRightEncoderOffset = -0.4912109375;
    private static final double kBackRightEncoderOffset = 0.2100390625;

    private static final double kBackRightXPosInches = -10.5;
    private static final double kBackRightYPosInches = -10.5;

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
            .createModuleConstants(
                    kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                    Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), true, false,
                    false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
            .createModuleConstants(
                    kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                    Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), false, false,
                    false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
            .createModuleConstants(
                    kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                    Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), false, false,
                    false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
            .createModuleConstants(
                    kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                    Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), true, false,
                    false);

    // private static final SwerveModuleConstants FrontLeft =
    // ConstantCreator.createModuleConstants(
    // kFrontLeftSteerMotorId,
    // kFrontLeftDriveMotorId,
    // kFrontLeftEncoderId,
    // kFrontLeftEncoderOffset,
    // Units.inchesToMeters(kFrontLeftXPosInches),
    // Units.inchesToMeters(kFrontLeftYPosInches),
    // // kInvertLeftSide
    // true
    // );
    // private static final SwerveModuleConstants FrontRight =
    // ConstantCreator.createModuleConstants(
    // kFrontRightSteerMotorId,
    // kFrontRightDriveMotorId,
    // kFrontRightEncoderId,
    // kFrontRightEncoderOffset,
    // Units.inchesToMeters(kFrontRightXPosInches),
    // Units.inchesToMeters(kFrontRightYPosInches),
    // // kInvertRightSide
    // false
    // );
    // private static final SwerveModuleConstants BackLeft =
    // ConstantCreator.createModuleConstants(
    // kBackLeftSteerMotorId,
    // kBackLeftDriveMotorId,
    // kBackLeftEncoderId,
    // kBackLeftEncoderOffset,
    // Units.inchesToMeters(kBackLeftXPosInches),
    // Units.inchesToMeters(kBackLeftYPosInches),
    // // kInvertLeftSide
    // false
    // );
    // private static final SwerveModuleConstants BackRight =
    // ConstantCreator.createModuleConstants(
    // kBackRightSteerMotorId,
    // kBackRightDriveMotorId,
    // kBackRightEncoderId,
    // kBackRightEncoderOffset,
    // Units.inchesToMeters(kBackRightXPosInches),
    // Units.inchesToMeters(kBackRightYPosInches),
    // // kInvertRightSide
    // true
    // );

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(
            DrivetrainConstants,
            FrontLeft,
            FrontRight,
            BackLeft,
            BackRight);

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules             Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
         *                                  drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz
         *                                  on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules);
        }
    }
}
