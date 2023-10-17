package frc.robot.Modes;

import frc.robot.Subsystems.SwerveDriveRobot;

public class DisabledMode {

    private SwerveDriveRobot swerveDrive;

    public DisabledMode(
            SwerveDriveRobot _swerveDrive) {
        swerveDrive = _swerveDrive;
    }

    /** This function is called when mode starts. */
    public void init() {
        swerveDrive.zeroMotors();
    }

    /** This function is called periodically. */
    public void periodic() {
    }
}
