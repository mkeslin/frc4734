package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
//import frc.robot.Subsystems.Elevator;
//import frc.robot.Commands.ElevatorDeployCommand;
//import frc.robot.Commands.ElevatorRaiseToAmpCommand;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.Commands.ShooterSetAngleCommand;

public class ShootAmpCommand extends SequentialCommandGroup {
    public final Limelight m_Limelight;
    public final PathPlanner m_PathPlanner;
    public final Intake m_Intake;
    public final Shooter m_Shooter;
    //public final Elevator m_Elevator;
    public ShooterSetAngleCommand shooterSetAngleCommand;


    public ShootAmpCommand(Limelight limelight, PathPlanner pathPlanner, Intake intake, Shooter shooter/*, Elevator elevator*/) {
        m_Limelight = limelight;
        m_PathPlanner = pathPlanner;
        m_Intake = intake;
        m_Shooter = shooter;
        //m_Elevator = elevator;
        shooterSetAngleCommand = new ShooterSetAngleCommand(m_Shooter, 8.5);
        shooterSetAngleCommand.setTarget(2.5);

        addCommands(
            // aim at speaker
            //
            //new ElevatorDeployCommand(m_Elevator, m_Elevator.getDeployVal()),
            //new ElevatorRaiseToAmpCommand(m_Elevator, 266),
            shooterSetAngleCommand,
            new ShootNoteCommand(m_Intake, m_Shooter, 0.5)
        );
    }
}
