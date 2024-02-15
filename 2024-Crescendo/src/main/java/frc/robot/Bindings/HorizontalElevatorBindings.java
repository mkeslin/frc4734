// package frc.robot.Bindings;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Controllers.ControllerButtons;
// import frc.robot.Subsystems.Elevator;

// public class HorizontalElevatorBindings {
//     public static void configureBindings(
//         CommandXboxController mechanismController,
//         Elevator horizontalElevator
//     ) {
//         mechanismController
//             .axisLessThan(ControllerButtons.CRY, -0.5)
//             .whileTrue(
//                 Commands.runOnce(
//                     () -> {
//                         horizontalElevator.movePositive();
//                     },
//                     horizontalElevator
//                 )
//             );
//         mechanismController
//             .axisGreaterThan(ControllerButtons.CRY, 0.5)
//             .whileTrue(
//                 Commands.runOnce(
//                     () -> {
//                         horizontalElevator.moveNegative();
//                     },
//                     horizontalElevator
//                 )
//             );

//         // A Button: reset position
//         mechanismController
//             .a()
//             .whileTrue(
//                 Commands.runOnce(
//                     () -> {
//                         horizontalElevator.zero();
//                     },
//                     horizontalElevator
//                 )
//             );
//     }
// }
