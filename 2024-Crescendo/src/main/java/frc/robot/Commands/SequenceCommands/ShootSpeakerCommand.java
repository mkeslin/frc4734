package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake;

public class ShootSpeakerCommand extends SequentialCommandGroup {
    public final Limelight m_ShooterLimelight;
    public final Limelight m_IntakeLimelight;
    public final PathPlanner m_PathPlanner;
    public final Shooter m_Shooter;
    public final Intake m_Intake;

    public ShootSpeakerCommand(Limelight shooterLimelight, Limelight intakeLimelight, PathPlanner pathPlanner, Intake intake, Shooter shooter) {
        m_ShooterLimelight = shooterLimelight;
        m_IntakeLimelight = intakeLimelight;
        m_PathPlanner = pathPlanner;
        m_Shooter = shooter;
        m_Intake = intake;

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
            new ShootNoteCommand(m_Intake, m_Shooter)

            //shooter.commandShoot(),
            //Commands.waitSeconds(2),
            // stop shooter
            //shooter.commandStop()
        );
    }
}
