package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;

public class ShootAmpCommand extends SequentialCommandGroup {

    public ShootAmpCommand(Limelight limelight, PathPlanner pathPlanner, Intake intake) {
        addCommands(
            // aim at speaker
            //
            Commands.waitSeconds(2),
            //
            // intake.commandShoot(),
            Commands.waitSeconds(2),
            intake.commandStop()
        );
    }
}
