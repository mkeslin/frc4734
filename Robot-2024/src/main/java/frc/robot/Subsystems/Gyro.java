package frc.robot.Subsystems;

// import com.ctre.phoenix.sensors.Pigeon2;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.I2C.Port;

public class Gyro {

    //private AHRS NavX;
    // private Pigeon2 gyro;
    private double pitch;
    private double roll;

    public Gyro() {
        pitch = 0;
        roll = 0;
        //gyro = new Pigeon2(61);
        //NavX = new AHRS(Port.kMXP);
        //NavX = new AHRS(Port.kOnboard); //kOnboard is the RIO's I2C port; change value if port changes
    }

    private void calculateGyro() {
        //pitch = gyro.getPitch();
        //roll = gyro.getRoll();
        //pitch = NavX.getPitch();
        //roll = NavX.getRoll();
    }

    public double getPitchValue() {
        return pitch;
    }

    public double getRollValue() {
        return roll;
    }

    public void putNums() {
        calculateGyro();
        //SmartDashboard.putBoolean("Gyro Connected", NavX.isConnected());
        //SmartDashboard.putBoolean("Gyro Calibrating", NavX.isCalibrating());
        SmartDashboard.putNumber("GP", pitch);
        SmartDashboard.putNumber("GR", roll);
    }
}
