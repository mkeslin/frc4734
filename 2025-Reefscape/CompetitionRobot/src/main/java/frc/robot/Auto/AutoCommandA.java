package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Commands.ElevatorDeployCommand;
//import frc.robot.Commands.ElevatorStowCommand;
// import frc.robot.Commands.IntakeDeployCommand;
import frc.robot.Commands.RobotCommands;
// import frc.robot.Commands.RobotRotateCommand;
// import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
// import frc.robot.Commands.SequenceCommands.ShootSpeakerCommand;
// import frc.robot.Commands.ShootNoteCommand;
// import frc.robot.Commands.ShooterSetAngleCommand;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.CoralSim.CoralSimLocation;
import frc.robot.Subsystems.CoralSim.CoralSimScoreLocation;
import frc.robot.Subsystems.Elevator;
// import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
// import frc.robot.Subsystems.Climber;
//import frc.robot.Subsystems.Elevator;
// import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.Shooter;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandA {
    public static AutoRoutine GDC(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, CoralSim coralSim) {
        PathPlannerPath Start_GPath = PathPlannerPath.fromPathFile("Start-G");
        PathPlannerPath G_PickupPath = PathPlannerPath.fromPathFile("G-Pickup");
        PathPlannerPath Pickup_DPath = PathPlannerPath.fromPathFile("Pickup-D");
        PathPlannerPath D_PickupPath = PathPlannerPath.fromPathFile("D-Pickup");
        PathPlannerPath Pickup_CPath = PathPlannerPath.fromPathFile("Pickup-C");

        Command command = Commands.sequence(
                RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)
                        .alongWith(Commands.waitSeconds(0.5).andThen(drivetrain.followPathCommand(Start_GPath))),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.G_L4,
                                coralSim))),
                drivetrain.followPathCommand(G_PickupPath)
                        .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim)),
                coralSim.setLocationCommand(CoralSimLocation.INTAKE),
                Commands.parallel(
                        Commands.waitSeconds(0.1)
                                .andThen(RobotCommands.intakeIntoScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)),
                        drivetrain.followPathCommand(Pickup_DPath)),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0).andThen(simulateCoral(CoralSimScoreLocation.D_L4,
                                coralSim))),
                drivetrain.followPathCommand(D_PickupPath)
                        .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim)),
                coralSim.setLocationCommand(CoralSimLocation.INTAKE),
                Commands.parallel(
                        Commands.waitSeconds(0)
                                .andThen(RobotCommands.intakeIntoScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)),
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
