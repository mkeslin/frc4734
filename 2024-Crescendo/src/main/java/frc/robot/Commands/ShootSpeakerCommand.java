package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.AprilTagAligner;
import frc.robot.Subsystems.Shooter;

public class ShootSpeakerCommand extends SequentialCommandGroup {

    public ShootSpeakerCommand(Limelight limelight, PathPlanner pathPlanner, Shooter shooter, AprilTagAligner aprilTagAligner) {
        addCommands(
            // rotate to speaker
            pathPlanner.moveToSpeaker(),
            Commands.waitSeconds(1),
            // align to tag
            aprilTagAligner.AlignToTag(1),
            Commands.waitSeconds(1),
            // adjust shooter
            Commands.waitSeconds(1),
            // shoot
            shooter.commandShoot(),
            Commands.waitSeconds(2),
            // stop shooter
            shooter.commandStop()
        );
    }
}
