package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
import frc.robot.Commands.SequenceCommands.ShootSpeakerCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
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
    private final Climber m_climber;
    private final Elevator m_elevator;
    private final Limelight m_intakeLimelight;
    //private final Limelight m_shooterLimelight;

    public AutoCommand(
        PathPlanner pathPlanner,
        Intake intake,
        Shooter shooter,
        Climber climber,
        Elevator elevator,
        Limelight intakeLimelight,
        //Limelight shooterLimelight,
        int[] noteOrder
    ) {
        m_pathPlanner = pathPlanner;
        m_intake = intake;
        m_shooter = shooter;
        m_climber = climber;
        m_elevator = elevator;
        m_intakeLimelight = intakeLimelight;
        //m_shooterLimelight = shooterLimelight;

        addRequirements(m_pathPlanner, m_intake, m_shooter, m_climber, m_elevator, m_intakeLimelight/*, m_shooterLimelight*/);

        // // load the commands for the specific notes
        // List<Command> commands = new ArrayList<Command>();
        // for (Integer noteNumber : noteOrder) {
        //     commands.add(moveAcquireShootCycle(noteNumber));
        // }
        // addCommands(commands.toArray(new Command[0]));

        // test
        addCommands(
            // start sequence
            // setStartConfiguration()
            // commands
            // m_pathPlanner.moveToTest1()
            moveAcquireShootCycle(2)
            // Commands.print("This is the auto command!!!!!!!!!!!!!!!")
        );
    }

    private Command setStartConfiguration() {
        // return Commands.sequence(
        //     Commands.parallel(
        //         // lower intake
        //         m_intake.commandDeploy(),
        //         // lower climbers
        //         m_climber.CommandFullRetract(),
        //         // retract elevator
        //         m_elevator.CommandFullRetract()
        //     ),
        //     // pivot elevator down
        //     m_elevator.CommandPivotStow()
        // );

        return Commands.sequence(
            // lower intake
            // m_intake.commandDeploy(),
            // lower climbers
            // m_climber.CommandFullRetract(),
            // retract elevator
            m_elevator.CommandFullRetract(),
            // pivot elevator down
            m_elevator.CommandPivotStow()
        );
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
        var shootSpeakerNoteCommand = new ShootSpeakerCommand(/*m_shooterLimelight,*/ m_intakeLimelight, m_pathPlanner, m_intake, m_shooter);

        return Commands.sequence(
            //Commands.print("Executing cycle for note " + noteNumber + "..."),
            // moveToNoteCommand,

            // m_pathPlanner.moveToOurNote2(),

            m_pathPlanner.moveToRedTest1(),
            m_pathPlanner.moveToRedTest2(),
            m_pathPlanner.moveToRedTest3(),
            m_pathPlanner.moveToRedTest4()

            // acquireNoteCommand,
            // shootSpeakerNoteCommand

            // m_intake.commandStartIn(),
            // Commands.waitSeconds(2),
            // m_intake.commandStopRoller(),
            // m_pathPlanner.moveToOurSpeaker()

            // m_pathPlanner.moveToTest4(),

            // shootSpeakerNoteCommand
            // m_shooter.commandShoot(),
            // Commands.waitSeconds(2),
            // m_shooter.commandStop()
            //Commands.print("...finished executing cycle for note " + noteNumber)
        );
    }
}
