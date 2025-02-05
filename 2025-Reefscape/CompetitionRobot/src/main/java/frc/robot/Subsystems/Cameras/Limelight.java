package frc.robot.Subsystems.Cameras;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

    //public static NetworkTable table; //check if static variable works; otherwise, use script from 2022
    //public static NetworkTable table2;

    public NetworkTable table;
    public String tableName;

    public Limelight(String t, int p) {
        table = NetworkTableInstance.getDefault().getTable(t);
        tableName = t;
        setPipeline(p);
    }

    /**
     * @return Whether or not limelight has targets within its field of view
     */
    public boolean hasTargets() {
        return LimelightHelpers.getTV(tableName);
    }

    /**
     * @return X value of limelight
     */
    public double getX() {
        return LimelightHelpers.getTX(tableName);
    }

    /**
     * @return Y value of limelight
     */
    public double getY() {
        return LimelightHelpers.getTY(tableName);
    }

    /*public static double getTargetYaw() {
        return(table.getEntry("camerapose_targetspace").getDoubleArray(new double[6])[5]);
    }*/

    /**
     * @return Area value of limelight
     */
    public double getArea() {
        return LimelightHelpers.getTA(tableName);
    }

    /**
     * @return Estimated x distance to target in meters
     */
    public double getXDistance() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(tableName).getX();
    }

    /**
     * @return Estimated y distance to target in meters
     */
    public double getYDistance() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(tableName).getY();
    }

    /**
     * @return yaw (rotation relative to y-axis) of Limelight camera target in radians
     */
    public double getYaw() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(tableName).getRotation().getY();
    }

    /**
     * Turns the limelight on and off
     * @param b true if on, false if off
     */
    public void status(boolean b) {
        if (b) {
            LimelightHelpers.setLEDMode_ForceOn(tableName);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(tableName);
        }
    }

    public double getPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(tableName);
    }

    public double getAprilTagID() {
        return LimelightHelpers.getFiducialID(tableName);
    }

    public void setPipeline(int p) {
        LimelightHelpers.setPipelineIndex(tableName, p);
    }

    public void putNums() {
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

    //////////////////////////////////////////
}
