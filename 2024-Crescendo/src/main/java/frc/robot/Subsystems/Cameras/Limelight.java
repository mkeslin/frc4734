package frc.robot.Subsystems.Cameras;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    public static NetworkTable table; //check if static variable works; otherwise, use script from 2022

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * @return X value of limelight
     */
    public static double getX() {
        return (table.getEntry("tx").getDouble(0.0));
    }

    /**
     * @return Y value of limelight
     */
    public static double getY() {
        return (table.getEntry("ty").getDouble(0.0));
    }

    /*public static double getTargetYaw() {
        return(table.getEntry("camerapose_targetspace").getDoubleArray(new double[6])[5]);
    }*/

    /**
     * @return Area value of limelight
     */
    public static double getArea() {
        return (table.getEntry("ta").getDouble(0.0));
    }

    /**
     * Turns the limelight on and off
     * @param b true if on, false if off
     */
    public static void status(boolean b) {
        if (b) {
            table.getEntry("ledMode").setNumber(3);
        } else {
            table.getEntry("ledMode").setNumber(1);
        }
    }

    public static double getPipeline() {
        return (table.getEntry("pipeline").getDouble(0));
    }

    public static void setPipeline(int p) {
        table.getEntry("pipeline").setNumber(p);
    }

    public static void putNums() {
        SmartDashboard.putNumber("lime-x", getX());
        SmartDashboard.putNumber("lime-y", getY());
        //SmartDashboard.putNumber("lime-target-yaw", getTargetYaw());
        SmartDashboard.putNumber("lime-area", getArea());
    }

    //////////////////////////////////////////

    public boolean canShootAmp() {
        return false;
    }

    public boolean canShootSpeaker() {
        return false;
    }

    public boolean canClimb() {
        return false;
    }
}
