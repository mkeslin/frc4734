package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PositionTracker;
import frc.robot.Commands.CenterToStationCommand;
import frc.robot.Commands.RobotCommands;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.CoralSim.CoralSimLocation;
import frc.robot.Subsystems.CoralSim.CoralSimScoreLocation;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandA {
    public static AutoRoutine GDC(
            PositionTracker positionTracker, 
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            Limelight stationLimelight,
            CoralSim coralSim) {
        PathPlannerPath Start_B = null;
        PathPlannerPath B_Pickup2 = null;

        PathPlannerPath Pickup2_D = null;
        PathPlannerPath D_Pickup2 = null;
        try {
            Start_B = PathPlannerPath.fromPathFile("Start-B");
            B_Pickup2 = PathPlannerPath.fromPathFile("B-Pickup2");

            Pickup2_D = PathPlannerPath.fromPathFile("Pickup2-D");
            D_Pickup2 = PathPlannerPath.fromPathFile("D-Pickup2");
        } catch (Exception exception) {
            DriverStation.reportError("[AutoCommandA]: " + exception.getMessage(), false);
        }

        var command = Commands.sequence(
                GetCycleCommand(Start_B, B_Pickup2, ScoreSide.Left, positionTracker, drivetrain, elevator, arm, sideToSide, lights,
                        reefLimelight, stationLimelight, coralSim),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Left, positionTracker, drivetrain, elevator, arm, sideToSide, lights,
                        reefLimelight, stationLimelight, coralSim),
                GetCycleCommand(Pickup2_D, D_Pickup2, ScoreSide.Right, positionTracker, drivetrain, elevator, arm, sideToSide, lights,
                        reefLimelight, stationLimelight, coralSim)
        //
        );

        return new AutoRoutine("GDC", command,
                List.of(Start_B, B_Pickup2, Pickup2_D, D_Pickup2),
                Start_B.getStartingDifferentialPose());
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
            Limelight stationLimelight,
            CoralSim coralSim) {
        var centerToStationCommand = new CenterToStationCommand(stationLimelight, drivetrain);

        Command command = Commands.sequence(
                // DRIVE TO REEF & POSITION CORAL
                // Commands.parallel(
                // drivetrain.moveToPose(new Pose2d()));
                Commands.waitSeconds(0.0)
                        .andThen(drivetrain.followPathCommand(pathToReef)),
                // Commands.waitSeconds(0.2)
                // .andThen(RobotCommands.movePostIntakeCoralCommand(elevator, arm, sideToSide, lights,
                // coralSim)))
                Commands.waitSeconds((0.0))
                        .andThen(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, scoreSide, true, drivetrain,
                                elevator,
                                arm, sideToSide, lights, reefLimelight, coralSim)),
                // //
                // // ),
                // SCORE
                RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, lights, coralSim),
                // Commands.run(() -> drivetrain.setRelativeSpeed(-0.5, 0, 0)).asProxy().withTimeout(0.45)
                // .andThen(Commands.runOnce(() -> drivetrain.setRelativeSpeed(0, 0, 0))),
                Commands.parallel(
                        // PRE-INTAKE
                        Commands.waitSeconds(0.4)
                                .andThen(RobotCommands.prepareIntakeCoralCommand(positionTracker, elevator, arm, sideToSide, coralSim)),
                        // DRIVE TO CORAL STATION
                        Commands.waitSeconds(0.0)
                                .andThen(drivetrain.followPathCommand(pathToCoralStation))
                //
                ),
                centerToStationCommand,
                // INTAKE
                Commands.waitSeconds(3.0).until(() -> positionTracker.getCoralInTray()),
                RobotCommands.returnToStartPositions(positionTracker, elevator, arm, sideToSide)
        //
        );

        return command;
    }

    public static Command simulateCoral(CoralSimScoreLocation scoreLocation, CoralSim coralSim) {
        return Commands.sequence(
                coralSim.setLocationCommand(CoralSimLocation.HIDDEN),
                coralSim.addScoringLocationCommand(scoreLocation));
    }
}
