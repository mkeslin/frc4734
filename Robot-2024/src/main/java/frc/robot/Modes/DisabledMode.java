package frc.robot.Modes;

import frc.robot.Subsystems.SwerveDrive;

public class DisabledMode {

    private SwerveDrive swerveDrive;

    public DisabledMode(
            SwerveDrive _swerveDrive) {
        swerveDrive = _swerveDrive;
    }

    public void init() {
        swerveDrive.zeroMotors();
    }

    /** This function is called periodically during operator control. */
    public void periodic() {
    }
}
