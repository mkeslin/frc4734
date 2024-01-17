package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Shooter;

public class ShootSpeakerNoteCommand extends SequentialCommandGroup {

    public ShootSpeakerNoteCommand(Limelight limelight, PathPlanner pathPlanner, Shooter shooter) {
        addCommands(
            // aim at speaker
            //
            Commands.waitSeconds(2),
            //
            shooter.commandShoot(),
            Commands.waitSeconds(2),
            shooter.commandStop()
        );
    }
}
