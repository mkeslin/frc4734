package frc.robot.Auto;

import java.util.List;
import java.util.Objects;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Logging.RobotLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.CenterToReefCommand;
// import frc.robot.Commands.RobotCommands; // Removed for 2026 - methods using deleted subsystems are commented out
import frc.robot.Commands.RobotContext;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandA {
    private static ScoreLevel m_autoScoreLevel = ScoreLevel.L4;

    public static AutoRoutine StartingPosition1(
            RobotContext context,
            CenterToReefCommand centerToReefCommand) {
        // Validate parameters
        Objects.requireNonNull(context, "RobotContext cannot be null");
        Objects.requireNonNull(centerToReefCommand, "CenterToReefCommand cannot be null");
        
        PathPlannerPath Start_A = null;
        PathPlannerPath A_Pickup1 = null;
        PathPlannerPath Pickup1_F = null;
        PathPlannerPath F_Pickup1 = null;
        
        try {
            Start_A = PathPlannerPath.fromPathFile("1-Start-A");
            A_Pickup1 = PathPlannerPath.fromPathFile("1-A-Pickup1");
            Pickup1_F = PathPlannerPath.fromPathFile("1-Pickup1-F");
            F_Pickup1 = PathPlannerPath.fromPathFile("1-F-Pickup1");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading paths for StartingPosition1: %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load paths for StartingPosition1: %s", exception.getMessage()), false);
        }
        
        // Validate paths were loaded successfully
        if (Start_A == null || A_Pickup1 == null || Pickup1_F == null || F_Pickup1 == null) {
            StringBuilder missingPaths = new StringBuilder();
            if (Start_A == null) missingPaths.append("1-Start-A ");
            if (A_Pickup1 == null) missingPaths.append("1-A-Pickup1 ");
            if (Pickup1_F == null) missingPaths.append("1-Pickup1-F ");
            if (F_Pickup1 == null) missingPaths.append("1-F-Pickup1 ");
            
            String errorMsg = String.format("[AutoCommandA] StartingPosition1: Failed to load paths: %s. Returning empty routine.", missingPaths.toString().trim());
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Routine 1 (Failed)", Commands.none());
        }
        
        var command = Commands.sequence(
                GetCycleCommand(Start_A, A_Pickup1, ScoreSide.Right, context, centerToReefCommand),
                GetCycleCommand(Pickup1_F, F_Pickup1, ScoreSide.Right, context, centerToReefCommand),
                GetCycleCommand(Pickup1_F, F_Pickup1, ScoreSide.Left, context, centerToReefCommand)
        );
        return new AutoRoutine("Routine 1", command,
                List.of(Start_A, A_Pickup1, Pickup1_F, F_Pickup1),
                Start_A.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPosition2(
            RobotContext context,
            CenterToReefCommand centerToReefCommand) {
        // Validate parameters
        Objects.requireNonNull(context, "RobotContext cannot be null");
        Objects.requireNonNull(centerToReefCommand, "CenterToReefCommand cannot be null");
        
        PathPlannerPath Start_B = null;
        PathPlannerPath B_Pickup2 = null;
        PathPlannerPath Pickup2_D = null;
        PathPlannerPath D_Pickup2 = null;
        
        try {
            Start_B = PathPlannerPath.fromPathFile("2-Start-B");
            B_Pickup2 = PathPlannerPath.fromPathFile("2-B-Pickup2");
            Pickup2_D = PathPlannerPath.fromPathFile("2-Pickup2-D");
            D_Pickup2 = PathPlannerPath.fromPathFile("2-D-Pickup2");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading paths for StartingPosition2: %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load paths for StartingPosition2: %s", exception.getMessage()), false);
        }
        
        // Validate paths were loaded successfully
        if (Start_B == null || B_Pickup2 == null || Pickup2_D == null || D_Pickup2 == null) {
            StringBuilder missingPaths = new StringBuilder();
            if (Start_B == null) missingPaths.append("2-Start-B ");
            if (B_Pickup2 == null) missingPaths.append("2-B-Pickup2 ");
            if (Pickup2_D == null) missingPaths.append("2-Pickup2-D ");
            if (D_Pickup2 == null) missingPaths.append("2-D-Pickup2 ");
            
            String errorMsg = String.format("[AutoCommandA] StartingPosition2: Failed to load paths: %s. Returning empty routine.", missingPaths.toString().trim());
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Routine 2 (Failed)", Commands.none());
        }
        
        var command = Commands.sequence(
                GetCycleCommand(Start_B, B_Pickup2, ScoreSide.Right, context, centerToReefCommand),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Right, context, centerToReefCommand),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Left, context, centerToReefCommand)
        );
        return new AutoRoutine("Routine 2", command,
                List.of(Start_B, B_Pickup2, Pickup2_D, D_Pickup2),
                Start_B.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPosition3(
            RobotContext context,
            CenterToReefCommand centerToReefCommand) {
        // Validate parameters
        Objects.requireNonNull(context, "RobotContext cannot be null");
        Objects.requireNonNull(centerToReefCommand, "CenterToReefCommand cannot be null");
        
        PathPlannerPath Start_C = null;
        PathPlannerPath C_Pickup2 = null;
        PathPlannerPath Pickup2_D = null;
        PathPlannerPath D_Pickup2 = null;
        
        try {
            Start_C = PathPlannerPath.fromPathFile("3-Start-C");
            C_Pickup2 = PathPlannerPath.fromPathFile("3-C-Pickup2");
            Pickup2_D = PathPlannerPath.fromPathFile("2-Pickup2-D");
            D_Pickup2 = PathPlannerPath.fromPathFile("2-D-Pickup2");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading paths for StartingPosition3: %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load paths for StartingPosition3: %s", exception.getMessage()), false);
        }
        
        // Validate paths were loaded successfully
        if (Start_C == null || C_Pickup2 == null || Pickup2_D == null || D_Pickup2 == null) {
            StringBuilder missingPaths = new StringBuilder();
            if (Start_C == null) missingPaths.append("3-Start-C ");
            if (C_Pickup2 == null) missingPaths.append("3-C-Pickup2 ");
            if (Pickup2_D == null) missingPaths.append("2-Pickup2-D ");
            if (D_Pickup2 == null) missingPaths.append("2-D-Pickup2 ");
            
            String errorMsg = String.format("[AutoCommandA] StartingPosition3: Failed to load paths: %s. Returning empty routine.", missingPaths.toString().trim());
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Routine 3 (Failed)", Commands.none());
        }
        
        var command = Commands.sequence(
                GetCycleCommand(Start_C, C_Pickup2, ScoreSide.Right, context, centerToReefCommand),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Right, context, centerToReefCommand),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Left, context, centerToReefCommand)
        );
        return new AutoRoutine("Routine 3", command,
                List.of(Start_C, C_Pickup2, Pickup2_D, D_Pickup2),
                Start_C.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPositionTuning1(CommandSwerveDrivetrain drivetrain) {
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("zzTuning-1");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading path 'zzTuning-1': %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load path 'zzTuning-1': %s", exception.getMessage()), false);
        }

        if (path == null) {
            String errorMsg = "[AutoCommandA] StartingPositionTuning1: Failed to load path 'zzTuning-1'. Returning empty routine.";
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Tuning1 (Failed)", Commands.none());
        }

        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("Tuning1", command, List.of(path), path.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPositionTuning2(CommandSwerveDrivetrain drivetrain) {
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("zzTuning-2");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading path 'zzTuning-2': %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load path 'zzTuning-2': %s", exception.getMessage()), false);
        }

        if (path == null) {
            String errorMsg = "[AutoCommandA] StartingPositionTuning2: Failed to load path 'zzTuning-2'. Returning empty routine.";
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Tuning2 (Failed)", Commands.none());
        }

        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("Tuning2", command, List.of(path), path.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPositionTuning3(CommandSwerveDrivetrain drivetrain) {
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("zzTuning-3");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading path 'zzTuning-3': %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load path 'zzTuning-3': %s", exception.getMessage()), false);
        }

        if (path == null) {
            String errorMsg = "[AutoCommandA] StartingPositionTuning3: Failed to load path 'zzTuning-3'. Returning empty routine.";
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Tuning3 (Failed)", Commands.none());
        }

        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("Tuning3", command, List.of(path), path.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPositionTuning4(CommandSwerveDrivetrain drivetrain) {
        Objects.requireNonNull(drivetrain, "CommandSwerveDrivetrain cannot be null");
        
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("zzTuning-4");
        } catch (Exception exception) {
            RobotLogger.logError(String.format("[AutoCommandA] ERROR: Exception loading path 'zzTuning-4': %s", exception.getMessage()));
            DriverStation.reportError(String.format("[AutoCommandA] Failed to load path 'zzTuning-4': %s", exception.getMessage()), false);
        }

        if (path == null) {
            String errorMsg = "[AutoCommandA] StartingPositionTuning4: Failed to load path 'zzTuning-4'. Returning empty routine.";
            RobotLogger.logError("ERROR: " + errorMsg);
            DriverStation.reportError(errorMsg, false);
            return new AutoRoutine("Tuning4 (Failed)", Commands.none());
        }

        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("Tuning4", command, List.of(path), path.getStartingDifferentialPose());
    }

    private static Command GetCycleCommand(
            PathPlannerPath pathToReef,
            PathPlannerPath pathToCoralStation,
            ScoreSide scoreSide,
            RobotContext context,
            CenterToReefCommand centerToReefCommand) {
        
        // Validate all parameters
        Objects.requireNonNull(pathToReef, "pathToReef cannot be null");
        Objects.requireNonNull(pathToCoralStation, "pathToCoralStation cannot be null");
        Objects.requireNonNull(scoreSide, "scoreSide cannot be null");
        Objects.requireNonNull(context, "RobotContext cannot be null");
        Objects.requireNonNull(centerToReefCommand, "CenterToReefCommand cannot be null");

        // REMOVED FOR 2026 - All RobotCommands methods that use deleted subsystems (Elevator, Arm, SideToSide) are commented out
        // The following commands are no longer available:
        // - RobotCommands.postIntakeCoralCommand(context)
        // - RobotCommands.prepareScoreCoralCommand(context, m_autoScoreLevel, scoreSide)
        // - RobotCommands.scoreCoralCommand(context)
        // - RobotCommands.preIntakeCoralCommand(context)
        // - RobotCommands.intakeCoralCommand(context)
        
        Command command = Commands.sequence(
                // DRIVE TO REEF & PRE-POSITION CORAL
                // DRIVE TO REEF
                Commands.waitSeconds(0.1)
                        .andThen(context.drivetrain.followPathCommand(pathToReef)),
                // PRE-POSITION CORAL - REMOVED FOR 2026
                // Commands.waitSeconds(0.0)
                //         .andThen(RobotCommands.postIntakeCoralCommand(context))
                // POSITION CORAL, CENTER, & SCORE
                Commands.sequence(
                        // POSITION CORAL - REMOVED FOR 2026
                        // RobotCommands.prepareScoreCoralCommand(context, m_autoScoreLevel, scoreSide),
                        // CENTER
                        centerToReefCommand,
                        // BACK UP A BIT
                        Commands.run(() -> context.drivetrain.setRelativeSpeed(-0.6, 0, 0))
                                .withTimeout(0.12)
                                .andThen(Commands.runOnce(() -> context.drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy(),
                        // SCORE - REMOVED FOR 2026
                        // RobotCommands.scoreCoralCommand(context),
                        // MOVE FORWARD - set coral if not completely placed
                        Commands.run(() -> context.drivetrain.setRelativeSpeed(0.75, 0, 0)).withTimeout(0.12)
                                .asProxy(),
                        // MOVE REVERSE
                        Commands.run(() -> context.drivetrain.setRelativeSpeed(-1, 0, 0)).withTimeout(0.35)
                                .asProxy()
                ),
                // PRE-INTAKE & DRIVE TO CORAL STATION
                // PRE-INTAKE - REMOVED FOR 2026
                // Commands.waitSeconds(0.0)
                //         .andThen(RobotCommands.preIntakeCoralCommand(context)),
                // DRIVE TO CORAL STATION
                Commands.waitSeconds(0.35)
                        .andThen(context.drivetrain.followPathCommand(pathToCoralStation)),
                // INTAKE
                Commands.waitSeconds(15.0).until(() -> context.positionTracker.getCoralInTray())
                        .andThen(Commands.waitSeconds(0.15)),
                // REMOVED FOR 2026:
                // RobotCommands.intakeCoralCommand(context)
                Commands.none() // Placeholder - replace with new intake command when available
        );

        return command;
    }
}
