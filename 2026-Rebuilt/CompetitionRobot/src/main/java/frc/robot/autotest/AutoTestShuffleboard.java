package frc.robot.autotest;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Utility class for setting up Shuffleboard layout for auto testing.
 */
public final class AutoTestShuffleboard {
    private AutoTestShuffleboard() {
        // Utility class - prevent instantiation
    }

    /**
     * Sets up the "AutoTest" Shuffleboard tab with all widgets.
     * 
     * @param harness The AutoTestHarness instance
     */
    public static void setupTab(AutoTestHarness harness) {
        ShuffleboardTab tab = Shuffleboard.getTab("AutoTest");
        
        // Start Pose chooser
        tab.add("Start Pose", harness.startPoseChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
        
        // Test chooser
        tab.add("Test Selection", harness.testChooser)
                .withPosition(2, 0)
                .withSize(3, 1);
        
        // Run toggle (NetworkTable boolean - already set up in harness)
        tab.add("Run Test", false)
                .withPosition(0, 1)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kToggleButton);
        
        // Stop toggle
        tab.add("Stop Test", false)
                .withPosition(1, 1)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kToggleButton);
        
        // Last Result - Name
        tab.addString("Last Result: Name", () -> {
            CommandRunResult last = harness.getLastResult();
            return last != null ? last.name() : "None";
        })
                .withPosition(0, 2)
                .withSize(2, 1);
        
        // Last Result - Reason
        tab.addString("Last Result: Reason", () -> {
            CommandRunResult last = harness.getLastResult();
            return last != null ? last.reason().toString() : "None";
        })
                .withPosition(2, 2)
                .withSize(2, 1);
        
        // Last Result - Duration
        tab.addNumber("Last Result: Duration (s)", () -> {
            CommandRunResult last = harness.getLastResult();
            return last != null ? last.durationSec() : 0.0;
        })
                .withPosition(4, 2)
                .withSize(1, 1);
        
        // Live Telemetry - Pose
        tab.addNumber("Live: poseX", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/poseX", 0.0))
                .withPosition(0, 3)
                .withSize(1, 1);
        
        tab.addNumber("Live: poseY", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/poseY", 0.0))
                .withPosition(1, 3)
                .withSize(1, 1);
        
        tab.addNumber("Live: headingErrorDeg", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/headingErrorDeg", 0.0))
                .withPosition(2, 3)
                .withSize(1, 1);
        
        // Live Telemetry - Vision
        tab.addNumber("Live: tagCount", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/tagCount", 0.0))
                .withPosition(0, 4)
                .withSize(1, 1);
        
        tab.addNumber("Live: tagAmbiguity", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/tagAmbiguity", 1.0))
                .withPosition(1, 4)
                .withSize(1, 1);
        
        // Live Telemetry - Shooter
        tab.addNumber("Live: shooterRpmActual", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/shooterRpmActual", 0.0))
                .withPosition(2, 4)
                .withSize(1, 1);
        
        tab.addNumber("Live: shooterAtSpeed", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/shooterAtSpeed", 0.0))
                .withPosition(3, 4)
                .withSize(1, 1);
        
        // Live Telemetry - Game pieces
        tab.addNumber("Live: ballCount", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/ballCount", -1.0))
                .withPosition(0, 5)
                .withSize(1, 1);
        
        tab.addNumber("Live: climbQualified", () -> 
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("AutoTest/Live/climbQualified", 0.0))
                .withPosition(1, 5)
                .withSize(1, 1);
    }
}
