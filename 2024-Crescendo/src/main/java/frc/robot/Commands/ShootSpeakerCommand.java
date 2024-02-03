package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.LimelightAligner;
import frc.robot.Subsystems.Shooter;

public class ShootSpeakerCommand extends SequentialCommandGroup {

    public ShootSpeakerCommand(Limelight shooterLimelight, Limelight intakeLimelight, PathPlanner pathPlanner, Shooter shooter, LimelightAligner limelightAligner) {
        addCommands(
            // rotate to speaker
            pathPlanner.moveToOurSpeaker(),
            Commands.waitSeconds(1),
            // align to tag
            // limelightAligner.alignToTag(1),
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
