package frc.robot.Bindings;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Controllers.ControllerButtons;
import frc.robot.Subsystems.Elevator;

public class VerticalElevatorBindings {

    public static void configureBindings(
        CommandXboxController mechanismController,
        Elevator verticalElevator
    ) {
        mechanismController
            .axisLessThan(ControllerButtons.CRY, -0.5)
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        verticalElevator.movePositive();
                    },
                    verticalElevator
                )
            );
        mechanismController
            .axisGreaterThan(ControllerButtons.CRY, 0.5)
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        verticalElevator.moveNegative();
                    },
                    verticalElevator
                )
            );
        mechanismController
            .a()
            .whileTrue(
                Commands.runOnce(
                    () -> {
                        verticalElevator.zero();
                    },
                    verticalElevator
                )
            );
    }
}
