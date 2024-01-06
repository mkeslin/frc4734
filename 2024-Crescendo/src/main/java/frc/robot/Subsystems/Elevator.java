package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    private TalonFX motor1;
    private TalonFX motor2;
    private boolean elevatorMovingIn, elevatorMovingOut;
    private int zeroValue, halfValue, fullValue;
    private double m1EncoderVal, m2EncoderVal;
    private String name;

    public Elevator(String n, int id1, int id2, int zero, int half, int full) {
        motor1 = new TalonFX(id1);
        motor2 = new TalonFX(id2);
        motor1.setSelectedSensorPosition(0);
        motor2.setSelectedSensorPosition(0);
        motor1.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);

        motor1.config_kD(0, 0.69, 0);
        motor2.config_kD(0, 0.69, 0);
        motor1.config_kI(0, 0.001, 0);
        motor2.config_kI(0, 0.001, 0);
        motor1.config_kF(0, 0.069, 0);
        motor2.config_kF(0, 0.069, 0);
        motor1.configMotionCruiseVelocity(739, 0);
        motor2.configMotionCruiseVelocity(739, 0);
        motor1.configMotionAcceleration(369, 0);
        motor2.configMotionAcceleration(369, 0);
        zeroValue = zero;
        halfValue = half;
        fullValue = full;
        name = n;
        m1EncoderVal = 0;
        m2EncoderVal = 0;
        elevatorMovingIn = false;
        elevatorMovingOut = false;
    }

    public void movePositive() {
        SmartDashboard.putNumber(name + "Elevator", getElevatorEncoderValue());
        elevatorMovingOut = true;
        elevatorMovingIn = false;
        if (Math.abs(getElevatorEncoderValue()) < Math.abs(halfValue)) {
            motor1.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.25);
            motor2.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.25);
        } else if (Math.abs(getElevatorEncoderValue()) < Math.abs(fullValue)) {
            motor1.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.1);
            motor2.set(ControlMode.PercentOutput, Math.signum(fullValue) * 0.1);
        } else {
            setZero(fullValue);
            elevatorMovingOut = false;
        }
    }

    public void moveNegative() {
        SmartDashboard.putNumber(name + "Elevator", getElevatorEncoderValue());
        elevatorMovingOut = false;
        elevatorMovingIn = true;
        if (Math.abs(getElevatorEncoderValue()) > Math.abs(halfValue)) {
            motor1.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.25);
            motor2.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.25);
        } else if (Math.abs(getElevatorEncoderValue()) > Math.abs(zeroValue)) {
            motor1.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.1);
            motor2.set(ControlMode.PercentOutput, -Math.signum(fullValue) * 0.1);
        } else {
            setZero(motor1.getSelectedSensorPosition());
            elevatorMovingIn = false;
        }
    }

    public void setZero(double z) {
        m1EncoderVal = z;
        m2EncoderVal = z;
    }

    public void zero() {
        if (Math.abs(m1EncoderVal - getElevatorEncoderValue()) > 200 && m1EncoderVal > 40000) {
            motor1.set(
                ControlMode.MotionMagic,
                m1EncoderVal,
                DemandType.ArbitraryFeedForward,
                0.069
            );
            motor2.set(
                ControlMode.MotionMagic,
                m2EncoderVal,
                DemandType.ArbitraryFeedForward,
                0.069
            );
        } else {
            motor1.set(ControlMode.PercentOutput, 0);
            motor2.set(ControlMode.PercentOutput, 0);
        }
        SmartDashboard.putNumber(name + "Elevator", getElevatorEncoderValue());
        SmartDashboard.putNumber(name + "Magic", m1EncoderVal);
    }

    public double getElevatorEncoderValue() {
        return motor1.getSelectedSensorPosition();
    }

    public boolean getElevatorMovingIn() {
        return elevatorMovingIn;
    }

    public boolean getElevatorMovingOut() {
        return elevatorMovingOut;
    }
}
