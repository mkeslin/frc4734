package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Elevator;
import frc.robot.Commands.ElevatorRaiseToAmpCommand;
import frc.robot.Commands.ShootNoteCommand;

public class ShootAmpCommand extends SequentialCommandGroup {
    public final Limelight m_Limelight;
    public final PathPlanner m_PathPlanner;
    public final Intake m_Intake;
    public final Shooter m_Shooter;
    public final Elevator m_Elevator;


    public ShootAmpCommand(Limelight limelight, PathPlanner pathPlanner, Intake intake, Shooter shooter, Elevator elevator) {
        m_Limelight = limelight;
        m_PathPlanner = pathPlanner;
        m_Intake = intake;
        m_Shooter = shooter;
        m_Elevator = elevator;

        addCommands(
            // aim at speaker
            //
            new ElevatorRaiseToAmpCommand(elevator, -54),
            m_Shooter.commandSetAngle(2.5),
            new ShootNoteCommand(m_Intake, m_Shooter, 0.5)
        );
    }
}
