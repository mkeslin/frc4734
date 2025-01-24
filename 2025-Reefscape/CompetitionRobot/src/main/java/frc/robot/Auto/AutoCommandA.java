package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.RobotCommands;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.CoralSim.CoralSimLocation;
import frc.robot.Subsystems.CoralSim.CoralSimScoreLocation;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.SideToSide;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandA {
    public static AutoRoutine GDC(
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        PathPlannerPath Start_GPath = null; // PathPlannerPath.fromPathFile("Start-G");
        PathPlannerPath G_PickupPath = null; // PathPlannerPath.fromPathFile("G-Pickup");
        PathPlannerPath Pickup_DPath = null; // PathPlannerPath.fromPathFile("Pickup-D");
        PathPlannerPath D_PickupPath = null; // PathPlannerPath.fromPathFile("D-Pickup");
        PathPlannerPath Pickup_CPath = null; // PathPlannerPath.fromPathFile("Pickup-C");

        Command command = Commands.sequence(
                RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, ScoreSide.Left, elevator, arm, sideToSide, coralSim)
                        .alongWith(Commands.waitSeconds(0.5).andThen(drivetrain.followPathCommand(Start_GPath))),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.G_L4,
                                coralSim))),
                drivetrain.followPathCommand(G_PickupPath)
                        .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, sideToSide, coralSim)),
                coralSim.setLocationCommand(CoralSimLocation.INTAKE),
                Commands.parallel(
                        Commands.waitSeconds(0.1)
                                .andThen(RobotCommands.intakeAndScoreCommand(ScoreLevel.L4, ScoreSide.Right, elevator, arm, sideToSide, coralSim)),
                        drivetrain.followPathCommand(Pickup_DPath)),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0).andThen(simulateCoral(CoralSimScoreLocation.D_L4,
                                coralSim))),
                drivetrain.followPathCommand(D_PickupPath)
                        .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, sideToSide, coralSim)),
                coralSim.setLocationCommand(CoralSimLocation.INTAKE),
                Commands.parallel(
                        Commands.waitSeconds(0)
                                .andThen(RobotCommands.intakeAndScoreCommand(ScoreLevel.L4, ScoreSide.Right, elevator, arm, sideToSide, coralSim)),
                        drivetrain.followPathCommand(Pickup_CPath)),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.C_L4,
                                coralSim))));

        return new AutoRoutine("GDC", command,
                List.of(Start_GPath, G_PickupPath, Pickup_DPath, D_PickupPath, Pickup_CPath),
                Start_GPath.getStartingDifferentialPose());
    }

    public static Command simulateCoral(CoralSimScoreLocation scoreLocation, CoralSim coralSim) {
        return Commands.sequence(
                coralSim.setLocationCommand(CoralSimLocation.HIDDEN),
                coralSim.addScoringLocationCommand(scoreLocation));
    }
}
