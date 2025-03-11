package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PositionTracker;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.CenterToStationCommand;
import frc.robot.Commands.RobotCommands;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandA {

    public static AutoRoutine StartingPosition1(
            PositionTracker positionTracker,
            CenterToReefCommand centerToReefCommand,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            Limelight stationLimelight) {
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
            DriverStation.reportError("[AutoCommandA]: " + exception.getMessage(), false);
        }
        var command = Commands.sequence(
                // GetDrivingPracticeCommand(Start_A, A_Pickup1, positionTracker, drivetrain, reefLimelight,
                // stationLimelight),
                // GetDrivingPracticeCommand(Pickup1_F, F_Pickup1, positionTracker, drivetrain, reefLimelight,
                // stationLimelight),
                // GetDrivingPracticeCommand(Pickup1_F, F_Pickup1, positionTracker, drivetrain, reefLimelight,
                // stationLimelight)

                GetCycleCommand(Start_A, A_Pickup1, ScoreSide.Left, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight),
                GetCycleCommand(Pickup1_F, F_Pickup1, ScoreSide.Left, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight),
                GetCycleCommand(Pickup1_F, F_Pickup1, ScoreSide.Right, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight)
        //
        );
        return new AutoRoutine("Routine 1", command,
                List.of(Start_A, A_Pickup1, Pickup1_F, F_Pickup1),
                Start_A.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPosition2(
            PositionTracker positionTracker,
            CenterToReefCommand centerToReefCommand,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            Limelight stationLimelight) {
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
            DriverStation.reportError("[AutoCommandA]: " + exception.getMessage(), false);
        }
        var command = Commands.sequence(
                // GetDrivingPracticeCommand(Start_B, B_Pickup2, positionTracker, drivetrain, reefLimelight,
                // stationLimelight),
                // GetDrivingPracticeCommand(Pickup2_D, D_Pickup2, positionTracker, drivetrain, reefLimelight,
                // stationLimelight),
                // GetDrivingPracticeCommand(Pickup2_D, D_Pickup2, positionTracker, drivetrain, reefLimelight,
                // stationLimelight)

                GetCycleCommand(Start_B, B_Pickup2, ScoreSide.Left, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Left, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Right, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight)
        //
        );
        return new AutoRoutine("Routine 2", command,
                List.of(Start_B, B_Pickup2, Pickup2_D, D_Pickup2),
                Start_B.getStartingDifferentialPose());
    }

    public static AutoRoutine StartingPosition3(
            PositionTracker positionTracker,
            CenterToReefCommand centerToReefCommand,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            Limelight stationLimelight) {
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
            DriverStation.reportError("[AutoCommandA]: " + exception.getMessage(), false);
        }
        var command = Commands.sequence(
                // GetDrivingPracticeCommand(Start_C, C_Pickup2, positionTracker, drivetrain, reefLimelight,
                // stationLimelight),
                // GetDrivingPracticeCommand(Pickup2_D, D_Pickup2, positionTracker, drivetrain, reefLimelight,
                // stationLimelight),
                // GetDrivingPracticeCommand(Pickup2_D, D_Pickup2, positionTracker, drivetrain, reefLimelight,
                // stationLimelight)

                GetCycleCommand(Start_C, C_Pickup2, ScoreSide.Left, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Left, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Right, positionTracker, drivetrain,
                        elevator, arm, sideToSide, lights, reefLimelight, stationLimelight)
        //
        );
        return new AutoRoutine("Routine 3", command,
                List.of(Start_C, C_Pickup2, Pickup2_D, D_Pickup2),
                Start_C.getStartingDifferentialPose());
    }

    private static Command GetDrivingPracticeCommand(
            PathPlannerPath pathToReef,
            PathPlannerPath pathToCoralStation,
            PositionTracker positionTracker,
            CommandSwerveDrivetrain drivetrain,
            Limelight reefLimelight,
            Limelight stationLimelight) {
        var centerToReefCommand = new CenterToReefCommand(reefLimelight, drivetrain, null);
        var centerToStationCommand = new CenterToStationCommand(positionTracker, stationLimelight, drivetrain, null);
        return Commands.sequence(
                drivetrain.followPathCommand(pathToReef),
                centerToReefCommand,
                drivetrain.followPathCommand(pathToCoralStation),
                centerToStationCommand
        //
        );
    }

    private static Command GetCycleCommand(
            PathPlannerPath pathToReef,
            PathPlannerPath pathToCoralStation,
            ScoreSide scoreSide,
            PositionTracker positionTracker,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            Limelight stationLimelight) {
        var centerToReefCommand = new CenterToReefCommand(reefLimelight, drivetrain, null);
        var centerToStationCommand = new CenterToStationCommand(positionTracker, stationLimelight, drivetrain, null);

        Command command = Commands.sequence(
                // DRIVE TO REEF & PRE-POSITION CORAL
                Commands.parallel(
                        // DRIVE TO REEF
                        Commands.waitSeconds(0.0)
                                .andThen(drivetrain.followPathCommand(pathToReef)),
                        // PRE-POSITION CORAL
                        Commands.waitSeconds(0.1)
                                .andThen(RobotCommands.postIntakeCoralCommand(positionTracker, elevator, arm,
                                        sideToSide, lights))
                //
                ),
                // POSITION CORAL, CENTER, & SCORE
                Commands.sequence(
                        Commands.parallel(
                                // POSITION CORAL
                                RobotCommands.prepareScoreCoralCommand(positionTracker, ScoreLevel.L4, scoreSide,
                                        drivetrain, elevator, arm, sideToSide, lights, reefLimelight),
                                // CENTER
                                centerToReefCommand
                        //
                        ),
                        // // POSITION CORAL
                        // RobotCommands.prepareScoreCoralCommand(positionTracker, ScoreLevel.L4, scoreSide, drivetrain,
                        // elevator, arm, sideToSide, lights, reefLimelight),
                        // // CENTER
                        // centerToReefCommand,
                        // SCORE
                        RobotCommands.scoreCoralCommand(positionTracker, drivetrain, elevator, arm, lights),
                        // MOVE FORWARD - set coral if not completely placed
                        Commands.run(() -> drivetrain.setRelativeSpeed(0.5, 0, 0)).withTimeout(0.15)
                                .andThen(Commands.runOnce(() -> drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy(),
                        // MOVE REVERSE
                        Commands.run(() -> drivetrain.setRelativeSpeed(-0.5, 0, 0)).withTimeout(0.65)
                                .andThen(Commands.runOnce(() -> drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                //
                ).until(() -> !positionTracker.getCoralInArm()),
                // PRE-INTAKE & DRIVE TO CORAL STATION
                Commands.parallel(
                        // PRE-INTAKE
                        Commands.waitSeconds(0.0)
                                .andThen(RobotCommands.preIntakeCoralCommand(positionTracker, elevator, arm, sideToSide,
                                        lights)),
                        // DRIVE TO CORAL STATION
                        Commands.waitSeconds(0.35)
                                .andThen(drivetrain.followPathCommand(pathToCoralStation))
                //
                ),
                centerToStationCommand,
                // INTAKE
                Commands.waitSeconds(15.0).until(() -> positionTracker.getCoralInTray()),
                RobotCommands.intakeCoralCommand(positionTracker, elevator, arm, sideToSide, lights)
        //
        );

        return command;
    }
}
