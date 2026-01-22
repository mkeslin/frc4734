package frc.robot.pitcheck;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages Shuffleboard UI for pit check testing.
 * 
 * Creates a "PitCheck" tab with:
 * - Controls: Run, Abort, RobotOnBlocks, ClimberMotionAllowed
 * - Status: Current state, step name, results
 * - Results table: All step results with timestamps
 */
public class PitCheckShuffleboard {
    private static final String TAB_NAME = "PitCheck";
    
    private final NetworkTable table;
    private final ShuffleboardTab tab;
    
    // Control entries
    private final BooleanEntry runPitCheckEntry;
    private final BooleanEntry abortPitCheckEntry;
    private final BooleanEntry robotOnBlocksEntry;
    private final BooleanEntry climberMotionAllowedEntry;
    
    // Status entries
    private final StringEntry currentStateEntry;
    private final StringEntry currentStepEntry;
    private final StringEntry lastStepResultEntry;
    private final StringEntry overallStatusEntry;
    
    // Step chooser for rerunning individual steps
    private final SendableChooser<String> stepChooser;
    private final BooleanEntry runSelectedStepEntry;

    public PitCheckShuffleboard() {
        table = NetworkTableInstance.getDefault().getTable(TAB_NAME);
        tab = Shuffleboard.getTab(TAB_NAME);
        
        // Controls section
        tab.add("Controls", false).withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(2, 1);
        
        runPitCheckEntry = table.getBooleanTopic("RunPitCheck").getEntry(false);
        tab.add("Run Pit Check", runPitCheckEntry)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 1)
            .withSize(2, 1);
        
        abortPitCheckEntry = table.getBooleanTopic("AbortPitCheck").getEntry(false);
        tab.add("Abort Pit Check", abortPitCheckEntry)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 1)
            .withSize(2, 1);
        
        robotOnBlocksEntry = table.getBooleanTopic("RobotOnBlocks").getEntry(false);
        tab.add("Robot On Blocks", robotOnBlocksEntry)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 2)
            .withSize(2, 1);
        
        climberMotionAllowedEntry = table.getBooleanTopic("ClimberMotionAllowed").getEntry(false);
        tab.add("Climber Motion Allowed", climberMotionAllowedEntry)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(2, 2)
            .withSize(2, 1);
        
        // Status section
        tab.add("Status", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 3).withSize(2, 1);
        
        currentStateEntry = table.getStringTopic("CurrentState").getEntry("IDLE");
        tab.add("Current State", currentStateEntry)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4)
            .withSize(2, 1);
        
        currentStepEntry = table.getStringTopic("CurrentStep").getEntry("-");
        tab.add("Current Step", currentStepEntry)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 4)
            .withSize(2, 1);
        
        lastStepResultEntry = table.getStringTopic("LastStepResult").getEntry("-");
        tab.add("Last Step Result", lastStepResultEntry)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 5)
            .withSize(4, 1);
        
        overallStatusEntry = table.getStringTopic("OverallStatus").getEntry("-");
        tab.add("Overall Status", overallStatusEntry)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 6)
            .withSize(4, 1);
        
        // Step rerun chooser
        stepChooser = new SendableChooser<>();
        stepChooser.setDefaultOption("None", "");
        tab.add("Selected Step", stepChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(4, 1)
            .withSize(2, 1);
        
        runSelectedStepEntry = table.getBooleanTopic("RunSelectedStep").getEntry(false);
        tab.add("Run Selected Step", runSelectedStepEntry)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(4, 2)
            .withSize(2, 1);
    }

    /**
     * Updates the UI with current pit check state.
     */
    public void update(PitCheckRunner.State state, String currentStep, 
                      PitCheckResult lastResult, PitCheckReport report) {
        // Update state
        currentStateEntry.set(state.toString());
        currentStepEntry.set(currentStep != null ? currentStep : "-");
        
        // Update last result
        if (lastResult != null) {
            lastStepResultEntry.set(lastResult.toString());
        } else {
            lastStepResultEntry.set("-");
        }
        
        // Update overall status
        if (report != null) {
            PitCheckReport.OverallStatus overall = report.getOverallStatus();
            String statusText = overall.toString();
            if (overall == PitCheckReport.OverallStatus.ABORTED && report.getAbortReason() != null) {
                statusText += ": " + report.getAbortReason();
            } else if (overall == PitCheckReport.OverallStatus.PASSED || 
                       overall == PitCheckReport.OverallStatus.WARNINGS || 
                       overall == PitCheckReport.OverallStatus.FAILED) {
                statusText += String.format(" (%.1fs)", report.getTotalDuration());
            }
            overallStatusEntry.set(statusText);
        } else {
            overallStatusEntry.set("-");
        }
    }

    /**
     * Adds step names to the chooser for rerunning individual steps.
     */
    public void addStepToChooser(String stepName) {
        stepChooser.addOption(stepName, stepName);
    }

    /**
     * Gets the selected step name from the chooser.
     */
    public String getSelectedStep() {
        return stepChooser.getSelected();
    }

    /**
     * Resets control toggles (call after reading them to prevent spamming).
     */
    public void resetControls() {
        runPitCheckEntry.set(false);
        abortPitCheckEntry.set(false);
        runSelectedStepEntry.set(false);
    }

    // Getters for control values
    public boolean getRunPitCheck() {
        return runPitCheckEntry.get(false);
    }

    public boolean getAbortPitCheck() {
        return abortPitCheckEntry.get(false);
    }

    public boolean getRobotOnBlocks() {
        return robotOnBlocksEntry.get(false);
    }

    public boolean getClimberMotionAllowed() {
        return climberMotionAllowedEntry.get(false);
    }

    public boolean getRunSelectedStep() {
        return runSelectedStepEntry.get(false);
    }
}
