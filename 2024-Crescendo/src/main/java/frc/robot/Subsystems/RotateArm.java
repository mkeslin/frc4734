package frc.robot.Subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateArm {

    private TalonFX armMotor;
    private boolean armMovingIn, armMovingOut;

    public RotateArm() {
        // armMotor = new TalonFX(ROTATEARMID);
        // armMotor.setSelectedSensorPosition(0);
        // armMotor.setNeutralMode(NeutralMode.Brake);
        // armMovingIn = false;
        // armMovingOut = false;
    }

    public void extend(double s) {
        armMovingOut = true;
        armMovingIn = false;
        // if (getArmEncoderValue() > -24000) {
        //     armMotor.set(ControlMode.PercentOutput, -0.25 * s);
        // } else if (getArmEncoderValue() > -35000) {
        //     armMotor.set(ControlMode.PercentOutput, -0.15 * s);
        // } else if (getArmEncoderValue() > -49000) {
        //     armMotor.set(ControlMode.PercentOutput, -0.1 * s);
        // } else {
        //     armMovingOut = false;
        //     zero();
        // }
    }

    public void retract(double s) {
        armMovingOut = false;
        armMovingIn = true;
        // if (getArmEncoderValue() < -20000) {
        //     armMotor.set(ControlMode.PercentOutput, 0.25 * s);
        // } else if (getArmEncoderValue() < -10000) {
        //     armMotor.set(ControlMode.PercentOutput, 0.15 * s);
        // } else if (getArmEncoderValue() < -1000) {
        //     armMotor.set(ControlMode.PercentOutput, 0.1 * s);
        // } else {
        //     armMovingIn = false;
        //     zero();
        // }
    }

    public void zero() {
        // armMotor.set(ControlMode.PercentOutput, 0);
        // SmartDashboard.putNumber("armEncoder", getArmEncoderValue());
    }

    public double getArmEncoderValue() {
        // return armMotor.getSelectedSensorPosition();
        return 0;
    }

    public boolean getArmMovingIn() {
        return armMovingIn;
    }

    public boolean getArmMovingOut() {
        return armMovingOut;
    }
}
