package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj.XboxController;
import frc.Controls.DriveController;
import frc.Controls.MechanismController;

public abstract class BaseSubsystem {
    protected DriveController driveController;
    protected MechanismController mechanismController;

    public BaseSubsystem(DriveController pDriveController)
    {
        driveController = pDriveController;
        // mechanismController = pMechanismController;
    }

    public BaseSubsystem(MechanismController pMechanismController)
    {
        // driveController = pDriveController;
        mechanismController = pMechanismController;
    }

    public BaseSubsystem(DriveController pDriveController, MechanismController pMechanismController)
    {
        driveController = pDriveController;
        mechanismController = pMechanismController;
    }

    // public void HandleController(XboxController driveController, XboxController mechanismController);
    public abstract void HandleController();
}
