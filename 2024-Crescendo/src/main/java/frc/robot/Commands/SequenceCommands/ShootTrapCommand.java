package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ShootTrapCommand extends SequentialCommandGroup {
    public final Limelight m_Limelight;
    public final PathPlanner m_PathPlanner;
    public final Intake m_Intake;
    public final Shooter m_Shooter;

    public ShootTrapCommand(Limelight limelight, PathPlanner pathPlanner, Intake intake, Shooter shooter) {
        m_Limelight = limelight;
        m_PathPlanner = pathPlanner;
        m_Intake = intake;
        m_Shooter = shooter;
        
        addCommands(
            // aim at speaker
            //
            new ShootNoteCommand(m_Intake, m_Shooter)
        );
    }
}
