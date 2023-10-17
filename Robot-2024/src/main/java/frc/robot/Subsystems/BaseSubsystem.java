package frc.robot.Subsystems;

import frc.robot.Controls.DriveController;
import frc.robot.Controls.MechanismController;

public abstract class BaseSubsystem {
    protected DriveController driveController;
    protected MechanismController mechanismController;

    public BaseSubsystem() {
    }

    public BaseSubsystem(DriveController pDriveController) {
        driveController = pDriveController;
    }

    public BaseSubsystem(MechanismController pMechanismController) {
        mechanismController = pMechanismController;
    }

    public BaseSubsystem(DriveController pDriveController, MechanismController pMechanismController) {
        driveController = pDriveController;
        mechanismController = pMechanismController;
    }

    public abstract void HandleController();
}
