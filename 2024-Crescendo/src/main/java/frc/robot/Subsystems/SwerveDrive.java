package frc.robot.Subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    private TalonFX FRD, FLD, BRD, BLD, FRS, FLS, BRS, BLS;
    private CANCoder FRE, FLE, BRE, BLE;
    private XboxController controller;
    private double Strafe, Forward, Rotation, WSFR, WSFL, WSBR, WSBL, WAFR, WAFL, WABR, WABL, FRA, FLA, BRA, BLA, fieldRelativeOffset;
    public double FieldRelative;
    private double SpeedFactor;

    /**
     * Creates a new SwerveDrive
     * @param c Driver controller
     */
    public SwerveDrive(XboxController c) {
        FRD = new TalonFX(FRDID);
        FLD = new TalonFX(FLDID);
        BRD = new TalonFX(BRDID);
        BLD = new TalonFX(BLDID);
        FRS = new TalonFX(FRSID);
        FLS = new TalonFX(FLSID);
        BRS = new TalonFX(BRSID);
        BLS = new TalonFX(BLSID);
        FRE = new CANCoder(FREID);
        FLE = new CANCoder(FLEID);
        BRE = new CANCoder(BREID);
        BLE = new CANCoder(BLEID);
        controller = c;
        fieldRelativeOffset = 0;
        SpeedFactor = 0.5;
        zeroMotors();
    }

    /**
     * Uses the controller to swerve
     */
    public void controllerDrive() {
        calculateEncoder();
        calculateGyro();
        Forward = -stickDrift(controller.getRawAxis(CLY));
        Strafe = stickDrift(controller.getRawAxis(CLX));
        Rotation = stickDrift(controller.getRawAxis(CRX) * 0.75);
        calculateSwerve();
        swerveDrive(FRD, FRS, WAFR, WSFR);
        swerveDrive(FLD, FLS, WAFL, WSFL);
        swerveDrive(BRD, BRS, WABR, WSBR);
        swerveDrive(BLD, BLS, WABL, WSBL);
    }

    /**
     *
     * @param f forward
     * @param s strafe
     * @param r rotation
     */
    public void drive(double f, double s, double r) {
        calculateEncoder();
        calculateGyro();
        Forward = f;
        Strafe = s;
        Rotation = r;
        SmartDashboard.putNumber("Forward", Forward);
        SmartDashboard.putNumber("Strafe", Strafe);
        SmartDashboard.putNumber("Rotation", Rotation);
        calculateSwerve();
        swerveDrive(FRD, FRS, WAFR, WSFR);
        swerveDrive(FLD, FLS, WAFL, WSFL);
        swerveDrive(BRD, BRS, WABR, WSBR);
        swerveDrive(BLD, BLS, WABL, WSBL);
    }

    /**
     * Calculates the speed and rotation for each module
     */
    private void calculateSwerve() {
        double hold = Forward;
        Forward = Forward * Math.cos(FieldRelative) + Strafe * Math.sin(FieldRelative);
        Strafe = Strafe * Math.cos(FieldRelative) - hold * Math.sin(FieldRelative);
        double A = Strafe - Rotation * WBL / WBR;
        double B = Strafe + Rotation * WBL / WBR;
        double C = Forward - Rotation * WBW / WBR;
        double D = Forward + Rotation * WBW / WBR;
        WSFR = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        WSFL = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        WSBR = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
        WSBL = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        WAFR = Math.toDegrees(Math.atan2(B, C));
        WAFL = Math.toDegrees(Math.atan2(B, D));
        WABR = Math.toDegrees(Math.atan2(A, C));
        WABL = Math.toDegrees(Math.atan2(A, D));
        double WSM = Math.max(Math.max(WSFR, WSFL), Math.max(WSBR, WSBL));
        if (WSM > 1.0) {
            WSFR = WSFR / WSM;
            WSFL = WSFL / WSM;
            WSBR = WSBR / WSM;
            WSBL = WSBL / WSM;
        }
        WAFR = optimizeSwerve(FRS, WAFR, FRA);
        WAFL = optimizeSwerve(FLS, WAFL, FLA);
        WABR = optimizeSwerve(BRS, WABR, BRA);
        WABL = optimizeSwerve(BLS, WABL, BLA);
    }

    /**
     * Optimizes the swerve angles to be between 0-90
     * @param dm drive motor
     * @param a angle
     * @param e encoder value
     */
    private double optimizeSwerve(TalonFX dm, double a, double e) {
        a = fixAngle(a + e) - 180;
        return a;
    }

    /**
     * Sets the speed and angle for a swerve module
     * @param dm drive motor
     * @param sm steer motor
     * @param a desired angle
     * @param s speed
     */
    private void swerveDrive(TalonFX dm, TalonFX sm, double a, double s) {
        if (Math.abs(s) * DTS > 0.05) dm.set(
            ControlMode.PercentOutput,
            s * DTS * SpeedFactor
        ); else dm.set(ControlMode.PercentOutput, 0.0);
        if (Math.abs(a) / 180 > 0.05) sm.set(ControlMode.PercentOutput, a / 180.0); else sm.set(
            ControlMode.PercentOutput,
            0.0
        );
    }

    /**
     * Sets the encoder values to FRA, FLA, BRA, and BLA
     */
    private void calculateEncoder() {
        FRA = fixAngle(FRE.getAbsolutePosition() - FREO);
        FLA = fixAngle(FLE.getAbsolutePosition() - FLEO);
        BRA = fixAngle(BRE.getAbsolutePosition() - BREO);
        BLA = fixAngle(BLE.getAbsolutePosition() - BLEO);
    }

    /**
     * Sets the gyro yaw to FieldRelative
     */
    private void calculateGyro() {
        FieldRelative = Math.toRadians(fixAngle(-fieldRelativeOffset));
    }

    /**
     * Fixes the angle to be between 0-360
     *
     * @param a The current angle
     * @return The value of the angle between 0-360
     */
    private double fixAngle(double a) {
        if (a > 360) a %= 360; else if (a < 0) a = 360 - (Math.abs(a) % 360);
        return a;
    }

    /**
     * Sets all motor speeds to 0.0
     */
    public void zeroMotors() {
        FRD.set(ControlMode.PercentOutput, 0.0);
        FLD.set(ControlMode.PercentOutput, 0.0);
        BRD.set(ControlMode.PercentOutput, 0.0);
        BLD.set(ControlMode.PercentOutput, 0.0);
        FRS.set(ControlMode.PercentOutput, 0.0);
        FLS.set(ControlMode.PercentOutput, 0.0);
        BRS.set(ControlMode.PercentOutput, 0.0);
        BLS.set(ControlMode.PercentOutput, 0.0);
        FRD.setInverted(false);
        FLD.setInverted(false);
        BRD.setInverted(false);
        BLD.setInverted(false);
    }

    /**
     * Negates stick drift
     * @param n number to remove stick drift
     * @return number incoorporating stick drift
     */
    private double stickDrift(double n) {
        if (n > -0.1 && n < 0.1) n = 0;
        return n;
    }

    /**
     * Resets the zero point for the gyro
     */
    public void reset0() {
        //fieldRelativeOffset = NavX.getYaw();
        SmartDashboard.putNumber("NavX", fieldRelativeOffset);
    }

    public void putNums() {
        calculateEncoder();
        SmartDashboard.putNumber("FL", FLA);
        SmartDashboard.putNumber("FR", FRA);
        SmartDashboard.putNumber("BL", BLA);
        SmartDashboard.putNumber("BR", BRA);
    }

    public double getSpeedFactor() {
        return SpeedFactor;
    }

    public void setSpeedFactor(double s) {
        SpeedFactor = s;
    }

    public void setMode(NeutralMode n) {
        FRD.setNeutralMode(n);
        FLD.setNeutralMode(n);
        BRD.setNeutralMode(n);
        BLD.setNeutralMode(n);
        FRS.setNeutralMode(n);
        FLS.setNeutralMode(n);
        BRS.setNeutralMode(n);
        BLS.setNeutralMode(n);
    }

    public double getSwervePosition() {
        return BLD.getSelectedSensorPosition();
    }
}
