package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Logging.RobotLogger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Handles scheduling of autos, odometry resetting, and display of paths to
 * NetworkTables. Provides an interface for a drive team to select an autonomous
 * routine, and preview the path that will be followed along with the robot's
 * estimated starting position, given any localization.
 * 
 * Instance-based design allows for better testability and dependency injection.
 * 
 * <p>
 * 
 * Use {@code init()} in {@code robotInit},
 * {@code periodicUpdate()} in {@code disabledPeriodic},
 * {@code runSelectedRoutine()} in {@code autonomousInit}, and
 * {@code endRoutine()} in {@code autonomousExit}.
 */
public class AutoManager {
    private HashMap<String, AutoRoutine> routines = new HashMap<String, AutoRoutine>();
    private AutoRoutine lastRoutine;
    public SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    private Field2d field = new Field2d();
    /**
     * The scheduled command without the additional composition (extra odometry
     * triggers, timer sets).
     */
    private Command baseCommand;
    /** The full scheduled command, used for exiting the routine. */
    private Command currentCommand;
    /** A consumer that takes in a Pose2d to reset the odometry of a drivetrain. */
    private Consumer<Pose2d> resetOdometryConsumer;
    private Timer timer = new Timer();


    /**
     * Creates a new AutoManager instance.
     */
    public AutoManager() {
    }

    /**
     * Initializes logging for the AutoManager.
     */
    public void init() {
    }

    /**
     * Add a routine to the manager.
     * 
     * @param routine the routine to add
     * @throws NullPointerException if routine is null
     */
    public void addRoutine(AutoRoutine routine) {
        Objects.requireNonNull(routine, "AutoRoutine cannot be null");
        String routineName = routine.getName();
        if (routineName == null || routineName.trim().isEmpty()) {
            String errorMsg = "[AutoManager] Cannot add routine with null or empty name";
            RobotLogger.logError(errorMsg);
            throw new IllegalArgumentException(errorMsg);
        }
        routines.put(routineName, routine);
        chooser.addOption(routineName, routine);
        RobotLogger.log(String.format("[AutoManager] Added auto routine: %s", routineName));
    }

    /**
     * Get the selected autonomous routine.
     * 
     * @return the selected autonomous routine
     */
    public AutoRoutine getSelectedRoutine() {
        return chooser.getSelected();
    }

    /**
     * Get the field object (used for the drivetrain to post odometry values).
     * 
     * @return the field object
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Set the function associated with resetting odometry. This function must be
     * set if you are running an autonomous routine with paths or have any
     * dependencies on localization. This function should take in a {@code Pose2d}
     * and set the robot's pose to that value.
     * 
     * @param consumer the {@code Pose2d} consumer
     * @throws NullPointerException if consumer is null
     */
    public void setResetOdometryConsumer(Consumer<Pose2d> consumer) {
        Objects.requireNonNull(consumer, "Reset odometry consumer cannot be null");
        this.resetOdometryConsumer = consumer;
        RobotLogger.log("[AutoManager] Reset odometry consumer set");
    }


    /**
     * Updates the NetworkTables field with the new selected auto path. This
     * should be put in {@code disabledPeriodic()}.
     */
    public void periodicUpdate() {
        AutoRoutine selectedRoutine = getSelectedRoutine();
        if (selectedRoutine == null) {
            // No routine selected, nothing to update
            return;
        }
        
        if (selectedRoutine != lastRoutine) {
            if (resetOdometryConsumer != null) {
                try {
                    resetOdometryConsumer.accept(selectedRoutine.getInitialPose());
                } catch (Exception e) {
                    RobotLogger.logError(String.format("[AutoManager] Exception resetting odometry: %s", e.getMessage()));
                    DriverStation.reportError(String.format("[AutoManager] Failed to reset odometry: %s", e.getMessage()), false);
                }
            }
            if (selectedRoutine.getPathPlannerPaths() != null && selectedRoutine.getPathPlannerPaths().size() > 0) {
                field.getObject("startingPose").setPose(selectedRoutine.getInitialPose());
                displayPaths(selectedRoutine.getPathPlannerPaths());
            }
            lastRoutine = selectedRoutine;
        }
    }

    /**
     * Display the selected routine's trajectories on the field object.
     * Currently not implemented - path visualization can be added here if needed.
     */
    private void displayPaths(List<PathPlannerPath> paths) {
        // Path visualization not currently implemented
    }

    /**
     * Schedules the selected routine. This should be run in
     * {@code autonomousInit()}.
     */
    public void runSelectedRoutine() {
        if (this.resetOdometryConsumer == null) {
            String errorMsg = "[AutoManager] No odometry reset consumer set. Autonomous may not work correctly.";
            RobotLogger.logError(errorMsg);
            DriverStation.reportError(errorMsg, false);
            return;
        }

        AutoRoutine selectedRoutine = this.getSelectedRoutine();
        if (selectedRoutine == null) {
            String errorMsg = "[AutoManager] No auto routine selected. Please choose an autonomous routine.";
            RobotLogger.logError(errorMsg);
            DriverStation.reportError(errorMsg, false);
            return;
        }

        try {
            // the line after this makes the autoroutine command a composition, and
            // commands that are in a composition cannot be recomposed, which is what this
            // would do if auto is run multiple times. this fixes it by removing the
            // composition from the scheduler.
            CommandScheduler.getInstance().removeComposedCommand(baseCommand);

            baseCommand = selectedRoutine.getCommand();
            if (baseCommand == null) {
                String errorMsg = String.format("[AutoManager] Selected routine '%s' has a null command.", selectedRoutine.getName());
                RobotLogger.logError(errorMsg);
                DriverStation.reportError(errorMsg, false);
                return;
            }

            timer.reset();
            timer.start();
            currentCommand = baseCommand
                    .beforeStarting(() -> {
                        try {
                            resetOdometryConsumer.accept(selectedRoutine.getInitialPose());
                        } catch (Exception e) {
                            RobotLogger.logError(String.format("[AutoManager] Exception resetting odometry before auto: %s", e.getMessage()));
                            DriverStation.reportError(String.format("[AutoManager] Failed to reset odometry: %s", e.getMessage()), false);
                        }
                    })
                    .andThen(Commands.runOnce(timer::stop))
                    .withName("auto");
            CommandScheduler.getInstance().schedule(currentCommand);
            RobotLogger.log(String.format("[AutoManager] Started autonomous routine: %s", selectedRoutine.getName()));
        } catch (Exception e) {
            String errorMsg = String.format("[AutoManager] ERROR: Exception starting autonomous routine '%s': %s", selectedRoutine.getName(), e.getMessage());
            RobotLogger.logError(errorMsg);
            DriverStation.reportError(errorMsg, false);
        }
    }

    /**
     * Ends the selected routine. This should be run in {@code teleopInit()}.
     */
    public void endRoutine() {
        timer.stop();
        if (currentCommand != null) {
            currentCommand.cancel();
        }
    }
}
