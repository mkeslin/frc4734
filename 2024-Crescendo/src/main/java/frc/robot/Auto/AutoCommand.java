package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
import frc.robot.Commands.SequenceCommands.ShootSpeakerCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import java.util.ArrayList;
import java.util.List;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommand extends SequentialCommandGroup {

    private final PathPlanner m_pathPlanner;
    private final Intake m_intake;
    private final Shooter m_shooter;
    // private final LimelightAligner m_limelightAligner;
    private final Limelight m_intakeLimelight;
    private final Limelight m_shooterLimelight;

    public AutoCommand(
        PathPlanner pathPlanner,
        Intake intake,
        Shooter shooter,
        Limelight intakeLimelight,
        Limelight shooterLimelight,
        int[] noteOrder
    ) {
        m_pathPlanner = pathPlanner;
        m_intake = intake;
        m_shooter = shooter;
        // m_limelightAligner = limelightAligner;
        m_intakeLimelight = intakeLimelight;
        m_shooterLimelight = shooterLimelight;

        addRequirements(m_pathPlanner, m_intake, m_shooter, m_intakeLimelight, m_shooterLimelight);

        // load the commands for the specific notes
        List<Command> commands = new ArrayList<Command>();
        for (Integer noteNumber : noteOrder) {
            commands.add(moveAcquireShootCycle(noteNumber));
        }
        addCommands(commands.toArray(new Command[0]));
    }

    private Command moveAcquireShootCycle(int noteNumber) {
        Command moveToNoteCommand;
        switch (noteNumber) {
            default:
            case 1:
                moveToNoteCommand = m_pathPlanner.moveToOurNote1();
                break;
            case 2:
                moveToNoteCommand = m_pathPlanner.moveToOurNote2();
                break;
            case 3:
                moveToNoteCommand = m_pathPlanner.moveToOurNote3();
                break;
            case 4:
                moveToNoteCommand = m_pathPlanner.moveToOurNote4();
                break;
            case 5:
                moveToNoteCommand = m_pathPlanner.moveToOurNote5();
                break;
            case 6:
                moveToNoteCommand = m_pathPlanner.moveToOurNote6();
                break;
            case 7:
                moveToNoteCommand = m_pathPlanner.moveToOurNote7();
                break;
            case 8:
                moveToNoteCommand = m_pathPlanner.moveToOurNote8();
                break;
        }

        var acquireNoteCommand = new AcquireNoteCommand(m_intakeLimelight, m_pathPlanner, m_intake);
        var shootSpeakerNoteCommand = new ShootSpeakerCommand(m_shooterLimelight, m_intakeLimelight, m_pathPlanner, m_intake, m_shooter);

        return Commands.sequence(
            //Commands.print("Executing cycle for note " + noteNumber + "..."),
            moveToNoteCommand,

            // acquireNoteCommand,
            // shootSpeakerNoteCommand
            
            m_intake.commandStartIn(),
            Commands.waitSeconds(1),
            m_intake.commandStopRoller(),

            m_pathPlanner.moveToOurSpeaker(),

            m_shooter.commandShoot(),
            Commands.waitSeconds(1),
            m_shooter.commandStop()

            //Commands.print("...finished executing cycle for note " + noteNumber)
        );
    }
}
