package frc.robot.Modes;

import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.RotateArm;
import frc.robot.Subsystems.SwerveDrive;
// import frc.robot.Subsystems.Cameras.Limelight;

public class TeleopMode implements IMode {

    private SwerveDrive swerveDrive;
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
            SwerveDrive _swerveDrive,
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

    /** This function is called once when teleop is enabled. */
    public void init() {
        swerveDrive.zeroMotors();
    }

    /** This function is called periodically during operator control. */
    public void periodic() {
        swerveDrive.HandleController();
        intake.HandleController();
        rotateArm.HandleController();
        horizontalElevator.HandleController();
        verticalElevator.HandleController();
    }
}
