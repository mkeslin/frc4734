package frc.robot.Modes;

import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.SwerveDrive;

public class TestMode {

    private Intake intake;

    public TestMode(
            Intake _intake) {
        intake = _intake;
    }

    public void init() {
        // Limelight.setPipeline(0);
        intake.enableCompressor();
    }

    /** This function is called periodically during operator control. */
    public void periodic() {
    }
}
