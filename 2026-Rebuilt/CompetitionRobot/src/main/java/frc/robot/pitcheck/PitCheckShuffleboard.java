package frc.robot.pitcheck;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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

    private final ShuffleboardTab tab;

    // Control entries (GenericEntry: Shuffleboard does not accept raw BooleanEntry)
    private final GenericEntry runPitCheckEntry;
    private final GenericEntry abortPitCheckEntry;
    private final GenericEntry robotOnBlocksEntry;
    private final GenericEntry climberMotionAllowedEntry;

    // Status entries
    private final GenericEntry currentStateEntry;
    private final GenericEntry currentStepEntry;
    private final GenericEntry lastStepResultEntry;
    private final GenericEntry overallStatusEntry;

    // Step chooser for rerunning individual steps
    private final SendableChooser<String> stepChooser;
    private final GenericEntry runSelectedStepEntry;

    public PitCheckShuffleboard() {
        tab = Shuffleboard.getTab(TAB_NAME);

        // Controls section
        tab.add("Controls", false).withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(2, 1);

        runPitCheckEntry = tab.add("Run Pit Check", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

        abortPitCheckEntry = tab.add("Abort Pit Check", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

        robotOnBlocksEntry = tab.add("Robot On Blocks", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();

        climberMotionAllowedEntry = tab.add("Climber Motion Allowed", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(2, 2)
                .withSize(2, 1)
                .getEntry();

        // Status section
        tab.add("Status", "").withWidget(BuiltInWidgets.kTextView).withPosition(0, 3).withSize(2, 1);

        currentStateEntry = tab.add("Current State", "IDLE")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 4)
                .withSize(2, 1)
                .getEntry();

        currentStepEntry = tab.add("Current Step", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 4)
                .withSize(2, 1)
                .getEntry();

        lastStepResultEntry = tab.add("Last Step Result", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 5)
                .withSize(4, 1)
                .getEntry();

        overallStatusEntry = tab.add("Overall Status", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 6)
                .withSize(4, 1)
                .getEntry();

        // Step rerun chooser
        stepChooser = new SendableChooser<>();
        stepChooser.setDefaultOption("None", "");
        tab.add("Selected Step", stepChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(4, 1)
                .withSize(2, 1);

        runSelectedStepEntry = tab.add("Run Selected Step", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(4, 2)
                .withSize(2, 1)
                .getEntry();
    }

    /**
     * Updates the UI with current pit check state.
     */
    public void update(PitCheckRunner.State state, String currentStep, 
                      PitCheckResult lastResult, PitCheckReport report) {
        // Update state
        currentStateEntry.setValue(NetworkTableValue.makeString(state.toString()));
        currentStepEntry.setValue(NetworkTableValue.makeString(currentStep != null ? currentStep : "-"));

        // Update last result
        if (lastResult != null) {
            lastStepResultEntry.setValue(NetworkTableValue.makeString(lastResult.toString()));
        } else {
            lastStepResultEntry.setValue(NetworkTableValue.makeString("-"));
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
            overallStatusEntry.setValue(NetworkTableValue.makeString(statusText));
        } else {
            overallStatusEntry.setValue(NetworkTableValue.makeString("-"));
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
        runPitCheckEntry.setValue(NetworkTableValue.makeBoolean(false));
        abortPitCheckEntry.setValue(NetworkTableValue.makeBoolean(false));
        runSelectedStepEntry.setValue(NetworkTableValue.makeBoolean(false));
    }

    // Getters for control values
    public boolean getRunPitCheck() {
        return runPitCheckEntry.getBoolean(false);
    }

    public boolean getAbortPitCheck() {
        return abortPitCheckEntry.getBoolean(false);
    }

    public boolean getRobotOnBlocks() {
        return robotOnBlocksEntry.getBoolean(false);
    }

    public boolean getClimberMotionAllowed() {
        return climberMotionAllowedEntry.getBoolean(false);
    }

    public boolean getRunSelectedStep() {
        return runSelectedStepEntry.getBoolean(false);
    }
}
