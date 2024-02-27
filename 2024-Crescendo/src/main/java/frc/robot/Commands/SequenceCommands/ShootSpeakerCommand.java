package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.PathPlanner.AprilTags.AprilTags;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ShootSpeakerCommand extends SequentialCommandGroup {
    public final Limelight m_shooterLimelight;
    public final Limelight m_intakeLimelight;
    public final PathPlanner m_pathPlanner;
    public final Shooter m_shooter;
    public final Intake m_intake;

    public ShootSpeakerCommand(Limelight shooterLimelight, Limelight intakeLimelight, PathPlanner pathPlanner, Intake intake, Shooter shooter) {
        m_shooterLimelight = shooterLimelight;
        m_intakeLimelight = intakeLimelight;
        m_pathPlanner = pathPlanner;
        m_shooter = shooter;
        m_intake = intake;

        addCommands(
            // move/rotate to speaker
            pathPlanner.moveToOurSpeaker(),
            // align to tag
            new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, m_intake, AprilTags.OurSpeakerLeft()),
            // adjust shooter
            // shooter.commandSetAngle(0),
            shooter.commandSetAngle(7.5),
            // shoot
            new ShootNoteCommand(m_intake, m_shooter, -1)
        );
    }
}
