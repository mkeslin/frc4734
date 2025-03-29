// package frc.robot.Auto;

// import java.util.List;

// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.PositionTracker;
// import frc.robot.Subsystems.Arm;
// import frc.robot.Subsystems.Elevator;
// import frc.robot.Subsystems.Lights;
// import frc.robot.Subsystems.SideToSide;
// import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

// /*
//  * Command that executes during autonomous mode
//  */
// public class AutoCommandTest2 {
//     public static AutoRoutine GDC(
//             PositionTracker positionTracker,
//             CommandSwerveDrivetrain drivetrain,
//             Elevator elevator,
//             Arm arm,
//             SideToSide sideToSide,
//             Lights lights,
//             VisionCamera reefCamera) {
//         PathPlannerPath Start_GPath = null; // PathPlannerPath.fromPathFile("Start-G");
//         PathPlannerPath G_PickupPath = null; // PathPlannerPath.fromPathFile("G-Pickup");
//         PathPlannerPath Pickup_DPath = null; // PathPlannerPath.fromPathFile("Pickup-D");
//         PathPlannerPath D_PickupPath = null; // PathPlannerPath.fromPathFile("D-Pickup");
//         PathPlannerPath Pickup_CPath = null; // PathPlannerPath.fromPathFile("Pickup-C");

//         Command command = Commands.sequence(
//                 // RobotCommands
//                 //         .prepareCoralScoreCommand(ScoreLevel.L4, ScoreSide.Left, false, drivetrain, elevator, arm,
//                 //                 sideToSide, lights, reefCamera, coralSim)
//                 //         .alongWith(Commands.waitSeconds(0.5).andThen(drivetrain.followPathCommand(Start_GPath))),
//                 // Commands.parallel(
//                 //         RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, lights, coralSim),
//                 //         Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.G_L4,
//                 //                 coralSim))),
//                 // drivetrain.followPathCommand(G_PickupPath)
//                 //         .alongWith(RobotCommands.prepareIntakeCoralCommand(positionTracker, elevator, arm, sideToSide,
//                 //                 coralSim)),
//                 // coralSim.setLocationCommand(CoralSimLocation.INTAKE),
//                 // Commands.parallel(
//                 //         Commands.waitSeconds(0.1)
//                 //                 .andThen(RobotCommands.intakeAndScoreCommand(ScoreLevel.L4, ScoreSide.Right, elevator,
//                 //                         arm, sideToSide, lights, coralSim)),
//                 //         drivetrain.followPathCommand(Pickup_DPath)),
//                 // Commands.parallel(
//                 //         RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, lights, coralSim),
//                 //         Commands.waitSeconds(0).andThen(simulateCoral(CoralSimScoreLocation.D_L4,
//                 //                 coralSim))),
//                 // drivetrain.followPathCommand(D_PickupPath)
//                 //         .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, sideToSide, coralSim)),
//                 // coralSim.setLocationCommand(CoralSimLocation.INTAKE),
//                 // Commands.parallel(
//                 //         Commands.waitSeconds(0)
//                 //                 .andThen(RobotCommands.intakeAndScoreCommand(ScoreLevel.L4, ScoreSide.Right, elevator,
//                 //                         arm, sideToSide, lights, coralSim)),
//                 //         drivetrain.followPathCommand(Pickup_CPath)),
//                 // Commands.parallel(
//                 //         RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, lights, coralSim),
//                 //         Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.C_L4,
//                 //                 coralSim)))
//         //
//         );

//         return new AutoRoutine("GDC", command,
//                 List.of(Start_GPath, G_PickupPath, Pickup_DPath, D_PickupPath, Pickup_CPath),
//                 Start_GPath.getStartingDifferentialPose());
//     }
// }
