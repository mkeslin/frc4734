package frc.robot.Modes;

import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.RotateArm;
import frc.robot.Subsystems.SwerveDrive;
import frc.robot.Subsystems.AutomaticSubsystems.Auto;
import frc.robot.Subsystems.Cameras.Limelight;

import static frc.robot.Constants.*;

public class AutonomousMode extends Auto implements IMode {
    // private SwerveDrive swerveDrive;
    // private Intake intake;
    // private RotateArm rotateArm;
    // private Elevator horizontalElevator;
    // private Elevator verticalElevator;
    // private Limelight limelight;
    // private Gyro gyro;

    // private Auto auto;

    public AutonomousMode(
            SwerveDrive _swerveDrive,
            Intake _intake,
            RotateArm _rotateArm,
            Elevator _horizontalElevator,
            Elevator _verticalElevator,
            Limelight _limelight,
            Gyro _gyro) {
        super(_swerveDrive, _intake, _rotateArm, _horizontalElevator, _verticalElevator, _limelight, _gyro);
        // Auto(swerveDrive);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    public void init() {
        swerve.zeroMotors();

        this.setMode(SCORE);
        this.initiateAuto();
    }

    /** This function is called periodically during autonomous. */
    public void periodic() {
        this.periodicAuto();
    }
}
