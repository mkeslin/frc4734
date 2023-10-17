package frc.robot.Modes;

import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.RotateArm;
import frc.robot.Subsystems.SwerveDriveRobot;
// import frc.robot.Subsystems.Cameras.Limelight;

public class TeleopMode implements IMode {

    private SwerveDriveRobot swerveDrive;
    private Intake intake;
    private RotateArm rotateArm;
    private Elevator horizontalElevator;
    private Elevator verticalElevator;
    // private Gyro gyro;
    // private LifeCam lifeCam;
    // private Limelight limelight;
    // private UsbCamera camera1;

    public TeleopMode(
            // Motors _motors,
            SwerveDriveRobot _swerveDrive,
            Intake _intake,
            RotateArm _rotateArm,
            Elevator _horizontalElevator,
            Elevator _verticalElevator) {
        swerveDrive = _swerveDrive;
        intake = _intake;
        rotateArm = _rotateArm;
        horizontalElevator = _horizontalElevator;
        verticalElevator = _verticalElevator;
    }

    /** This function is called when mode starts. */
    public void init() {
        swerveDrive.zeroMotors();
    }

    /** This function is called periodically. */
    public void periodic() {
        swerveDrive.HandleController();
        intake.HandleController();
        rotateArm.HandleController();
        horizontalElevator.HandleController();
        verticalElevator.HandleController();
    }
}
