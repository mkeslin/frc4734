package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.PathPlanner.AprilTags.AprilTags;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ShootSpeakerCommand extends SequentialCommandGroup {
    public ShootSpeakerCommand(Limelight shooterLimelight, Limelight intakeLimelight, PathPlanner pathPlanner, Intake intake, Shooter shooter) {
        addCommands(
            // move/rotate to speaker
            Commands.print("-> Move to speaker..."),
            pathPlanner.moveToOurSpeaker(),
            // align to tag
            // new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, m_intake, AprilTags.OurSpeakerLeft()),
            // adjust shooter
            // shooter.commandSetAngle(7.5),
            // shoot
            Commands.print("-> Shoot note..."),
            new ShootNoteCommand(intake, shooter, .1)
        );
    }
}
