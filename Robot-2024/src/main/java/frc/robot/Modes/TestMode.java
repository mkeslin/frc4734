package frc.robot.Modes;

import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.SwerveDrive;

public class TestMode {

    private Intake intake;

    public TestMode(
            Intake _intake) {
        intake = _intake;
    }

    /** This function is called when mode starts. */
    public void init() {
        // Limelight.setPipeline(0);
        intake.enableCompressor();
    }

    /** This function is called periodically. */
    public void periodic() {
    }
}
