package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.Commands.ShooterSetAngleCommand;
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

        var shooterSetAngleCommandHigh = new ShooterSetAngleCommand(shooter, Shooter.MAX_PIVOT_ENCODER_VAL);
        shooterSetAngleCommandHigh.setTarget(Shooter.AUTO_SPEAKER_PIVOT_ENCODER_VAL);

        var shooterSetAngleCommandLow = new ShooterSetAngleCommand(shooter, Shooter.MAX_PIVOT_ENCODER_VAL);
        shooterSetAngleCommandLow.setTarget(0);

        addCommands(
            // move/rotate to speaker
            Commands.print("-> Move to speaker..."),
            pathPlanner.moveToOurSpeaker(),
            // align to tag
            // new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, m_intake, AprilTags.OurSpeakerLeft()),
            // raise shooter angle
            Commands.print("-> Raise shooter angle..."),
            shooterSetAngleCommandHigh,
            // shoot
            Commands.print("-> Shoot note..."),
            new ShootNoteCommand(intake, shooter, 1.0),
            // lower shoot angle
            Commands.print("-> Lower shooter angle..."),
            shooterSetAngleCommandLow
        );
    }
}
