package frc.robot.autotest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auto.commands.AutoConstants;
import frc.robot.Auto.commands.CmdStopAll;
import frc.robot.Auto.commands.StartPoseId;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Test harness for autonomous command testing.
 * 
 * <p>Provides a framework for testing:
 * <ul>
 *   <li><b>Atoms</b>: Individual atomic commands (e.g., CmdFollowPath, CmdSnapToHeading)</li>
 *   <li><b>Molecules</b>: Short 2-3 command sequences (e.g., Seed->Path->Aim)</li>
 *   <li><b>Full Autos</b>: Complete autonomous routines</li>
 * </ul>
 * 
 * <p><b>Testing Ladder:</b>
 * <ol>
 *   <li>Start with <b>Atoms</b> - test individual commands in isolation to verify basic functionality</li>
 *   <li>Move to <b>Molecules</b> - test 2-3 command sequences to verify composition works</li>
 *   <li>Finally test <b>Full Autos</b> - test complete routines end-to-end</li>
 * </ol>
 * 
 * <p><b>How to Run Tests from Driver Station:</b>
 * <ol>
 *   <li>Open Shuffleboard "AutoTest" tab</li>
 *   <li>Select start pose (LEFT, CENTER, RIGHT)</li>
 *   <li>Select test from chooser (atoms, molecules, or full autos)</li>
 *   <li>Toggle "Run" to true (or press button if using button binding)</li>
 *   <li>Monitor live telemetry and last result</li>
 *   <li>If test fails, check LastResult for reason (TIMEOUT, CONDITION_FALSE, etc.)</li>
 *   <li>Use "Stop" toggle to emergency stop if needed</li>
 * </ol>
 * 
 * <p><b>What to Check in Telemetry When a Test Fails:</b>
 * <ul>
 *   <li><b>TIMEOUT</b>: Check if command is too slow, or timeout too short</li>
 *   <li><b>CONDITION_FALSE</b>: Check vision quality (tagCount, tagAmbiguity), pose accuracy, shooter RPM</li>
 *   <li><b>INTERRUPTED</b>: Check if Stop was pressed or command was canceled</li>
 *   <li><b>ERROR</b>: Check for exceptions in logs, verify subsystems are initialized</li>
 *   <li>Compare metricsStart vs metricsEnd to see what changed</li>
 *   <li>Check headingErrorDeg, poseX/Y accuracy, tagCount/ambiguity for vision issues</li>
 * </ul>
 */
public class AutoTestHarness {
    private final Map<String, Supplier<Command>> atomFactories = new HashMap<>();
    private final Map<String, Supplier<Command>> moleculeFactories = new HashMap<>();
    public final SendableChooser<String> testChooser = new SendableChooser<>();
    public final SendableChooser<StartPoseId> startPoseChooser = new SendableChooser<>();
    private final List<CommandRunResult> resultHistory = new ArrayList<>();
    private final AtomicBoolean isTestRunning = new AtomicBoolean(false);
    
    // Subsystems (required for telemetry)
    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonVision vision;
    private final Shooter shooter;
    private final Feeder feeder;
    private final DeployableIntake intake;
    private final Climber climber;
    private final PositionTracker positionTracker;
    
    // NetworkTable entries for Run/Stop toggles
    private final NetworkTable testTable;
    private final BooleanEntry runEntry;
    private final BooleanEntry stopEntry;
    private boolean lastRunValue = false;
    private boolean lastStopValue = false;
    
    private Command currentTestCommand;

    /**
     * Creates a new AutoTestHarness.
     * 
     * @param drivetrain Drivetrain subsystem (required)
     * @param vision Vision subsystem (can be null)
     * @param shooter Shooter subsystem (can be null)
     * @param feeder Feeder subsystem (can be null)
     * @param intake Intake subsystem (can be null)
     * @param climber Climber subsystem (can be null)
     * @param positionTracker Position tracker (can be null)
     * @throws NullPointerException if drivetrain is null
     */
    public AutoTestHarness(
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        this.vision = vision;
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.climber = climber;
        this.positionTracker = positionTracker;
        
        // Use same path as Shuffleboard tab so Run/Stop widgets and harness share the same entries (path: Shuffleboard/AutoTest/Run, .../Stop)
        testTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("AutoTest");
        runEntry = testTable.getBooleanTopic("Run").getEntry(false);
        stopEntry = testTable.getBooleanTopic("Stop").getEntry(false);
    }

    /**
     * Registers an atom (single command) test.
     * 
     * @param name Display name for the test
     * @param factory Supplier that creates the command (called when test is selected)
     */
    public void registerAtom(String name, Supplier<Command> factory) {
        Objects.requireNonNull(name, "name cannot be null");
        Objects.requireNonNull(factory, "factory cannot be null");
        atomFactories.put("TEST: Atom - " + name, factory);
    }

    /**
     * Registers a molecule (2-3 command sequence) test.
     * 
     * @param name Display name for the test
     * @param factory Supplier that creates the command (called when test is selected)
     */
    public void registerMolecule(String name, Supplier<Command> factory) {
        Objects.requireNonNull(name, "name cannot be null");
        Objects.requireNonNull(factory, "factory cannot be null");
        moleculeFactories.put("TEST: Molecule - " + name, factory);
    }

    /**
     * Registers a full auto routine.
     * 
     * @param name Display name for the auto
     * @param factory Supplier that creates the command
     */
    public void registerAuto(String name, Supplier<Command> factory) {
        Objects.requireNonNull(name, "name cannot be null");
        Objects.requireNonNull(factory, "factory cannot be null");
        testChooser.addOption("AUTO: " + name, "AUTO: " + name);
        // Store in a separate map or handle differently
        // For now, we'll use a combined approach
    }

    /**
     * Initializes the test harness.
     * Sets up choosers and publishes them to SmartDashboard.
     */
    public void initialize() {
        // Setup start pose chooser
        startPoseChooser.setDefaultOption("LEFT", StartPoseId.POS_1);
        startPoseChooser.addOption("CENTER", StartPoseId.POS_2);
        startPoseChooser.addOption("RIGHT", StartPoseId.POS_3);
        
        // Add atoms to chooser
        for (String name : atomFactories.keySet()) {
            testChooser.addOption(name, name);
        }
        
        // Add molecules to chooser
        for (String name : moleculeFactories.keySet()) {
            testChooser.addOption(name, name);
        }
        
        // Publish choosers
        SmartDashboard.putData("AutoTest/StartPose", startPoseChooser);
        SmartDashboard.putData("AutoTest/TestChooser", testChooser);
        
        // Initialize NetworkTable entries
        runEntry.set(false);
        stopEntry.set(false);
    }

    /**
     * Periodic update - checks for Run/Stop toggles and updates live telemetry.
     * Call this from robotPeriodic() or disabledPeriodic().
     */
    public void periodic() {
        // Check Run toggle
        boolean runValue = runEntry.get();
        if (runValue && !lastRunValue && !isTestRunning.get()) {
            // Run toggled from false to true, and no test is running
            startTest();
        }
        lastRunValue = runValue;
        
        // Check Stop toggle
        boolean stopValue = stopEntry.get();
        if (stopValue && !lastStopValue) {
            // Stop toggled from false to true
            stopTest();
        }
        lastStopValue = stopValue;
        
        // Update live telemetry
        updateLiveTelemetry();
        
        // Check if test finished
        if (isTestRunning.get() && currentTestCommand != null && currentTestCommand.isFinished()) {
            isTestRunning.set(false);
            currentTestCommand = null;
            // Reset Run toggle
            runEntry.set(false);
        }
    }

    /**
     * Builds the selected test command.
     * 
     * @return The command for the selected test, or null if none selected
     */
    public Command buildSelectedTestCommand() {
        String selectedTest = testChooser.getSelected();
        if (selectedTest == null) {
            return null;
        }
        
        // Check if it's an atom
        if (atomFactories.containsKey(selectedTest)) {
            return atomFactories.get(selectedTest).get();
        }
        
        // Check if it's a molecule
        if (moleculeFactories.containsKey(selectedTest)) {
            return moleculeFactories.get(selectedTest).get();
        }
        
        return null;
    }

    /**
     * Starts the selected test.
     * Prevents re-triggering if a test is already running.
     */
    public void startTest() {
        if (isTestRunning.get()) {
            // Test already running - ignore
            return;
        }
        
        Command testCommand = buildSelectedTestCommand();
        if (testCommand == null) {
            SmartDashboard.putString("AutoTest/Status", "No test selected");
            return;
        }
        
        // Wrap with LoggedCommand for telemetry
        Command loggedCommand = LoggedCommand.logWrap(
                testChooser.getSelected(),
                testCommand,
                AutoConstants.DEFAULT_PATH_TIMEOUT, // Default timeout
                null, // No heading target by default
                drivetrain, vision, shooter, feeder, intake, climber, positionTracker,
                resultHistory);
        
        currentTestCommand = loggedCommand;
        isTestRunning.set(true);
        CommandScheduler.getInstance().schedule(loggedCommand);
        SmartDashboard.putString("AutoTest/Status", "Test running: " + testChooser.getSelected());
    }

    /**
     * Stops the current test and schedules CmdStopAll.
     */
    public void stopTest() {
        if (currentTestCommand != null) {
            currentTestCommand.cancel();
            currentTestCommand = null;
        }
        
        // Schedule stop all command
        Command stopAll = CmdStopAll.create(drivetrain, shooter, feeder, intake);
        CommandScheduler.getInstance().schedule(stopAll);
        
        isTestRunning.set(false);
        runEntry.set(false);
        SmartDashboard.putString("AutoTest/Status", "Test stopped");
    }

    /**
     * Gets the last test result.
     * 
     * @return The most recent CommandRunResult, or null if no tests have run
     */
    public CommandRunResult getLastResult() {
        if (resultHistory.isEmpty()) {
            return null;
        }
        return resultHistory.get(resultHistory.size() - 1);
    }

    /**
     * Gets the result history.
     * 
     * @return List of last 20 test results
     */
    public List<CommandRunResult> getResultHistory() {
        return new ArrayList<>(resultHistory);
    }

    /**
     * Gets the selected start pose.
     * 
     * @return The selected StartPoseId, or POS_1 as default
     */
    public StartPoseId getSelectedStartPose() {
        StartPoseId selected = startPoseChooser.getSelected();
        return selected != null ? selected : StartPoseId.POS_1;
    }

    /**
     * Updates live telemetry values on SmartDashboard.
     */
    private void updateLiveTelemetry() {
        Map<String, Double> metrics = TelemetrySnapshot.capture(
                drivetrain, vision, shooter, feeder, intake, climber, positionTracker,
                Optional.empty());
        
        SmartDashboard.putNumber("AutoTest/Live/poseX", metrics.getOrDefault("poseX", 0.0));
        SmartDashboard.putNumber("AutoTest/Live/poseY", metrics.getOrDefault("poseY", 0.0));
        SmartDashboard.putNumber("AutoTest/Live/headingErrorDeg", metrics.getOrDefault("headingErrorDeg", 0.0));
        SmartDashboard.putNumber("AutoTest/Live/tagCount", metrics.getOrDefault("tagCount", 0.0));
        SmartDashboard.putNumber("AutoTest/Live/tagAmbiguity", metrics.getOrDefault("tagAmbiguity", 1.0));
        SmartDashboard.putNumber("AutoTest/Live/shooterRpmActual", metrics.getOrDefault("shooterRpmActual", 0.0));
        SmartDashboard.putNumber("AutoTest/Live/shooterAtSpeed", metrics.getOrDefault("shooterAtSpeed", 0.0));
        SmartDashboard.putNumber("AutoTest/Live/ballCount", metrics.getOrDefault("ballCount", -1.0));
        SmartDashboard.putNumber("AutoTest/Live/climbQualified", metrics.getOrDefault("climbQualified", 0.0));
    }
}
