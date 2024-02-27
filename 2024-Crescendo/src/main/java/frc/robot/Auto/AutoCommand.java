package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ElevatorDeployCommand;
import frc.robot.Commands.ElevatorStowCommand;
import frc.robot.Commands.IntakeDeployCommand;
import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
import frc.robot.Commands.SequenceCommands.ShootSpeakerCommand;
import frc.robot.Commands.ShootNoteCommand;
import frc.robot.Commands.ShooterSetAngleCommand;
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
    private final Limelight m_shooterLimelight;

    public AutoCommand(
        PathPlanner pathPlanner,
        Intake intake,
        Shooter shooter,
        Climber climber,
        Elevator elevator,
        Limelight intakeLimelight,
        Limelight shooterLimelight,
        int[] noteOrder
    ) {
        m_pathPlanner = pathPlanner;
        m_intake = intake;
        m_shooter = shooter;
        m_climber = climber;
        m_elevator = elevator;
        m_intakeLimelight = intakeLimelight;
        m_shooterLimelight = shooterLimelight;

        addRequirements(m_pathPlanner, m_intake, m_shooter, m_climber, m_elevator, m_intakeLimelight, m_shooterLimelight);

        // // load the commands for the specific notes
        // List<Command> commands = new ArrayList<Command>();
        // for (Integer noteNumber : noteOrder) {
        //     commands.add(moveAcquireShootCycle(noteNumber));
        // }
        // addCommands(commands.toArray(new Command[0]));

        // test
        addCommands(
            // start sequence
            shootPreloadedNote(),
            // commands
            moveAcquireShootCycle(2)
        );
    }

    private Command shootPreloadedNote() {
        var elevatorDeployCommand = new ElevatorDeployCommand(m_elevator, 20);
        var intakeDeployCommand = new IntakeDeployCommand(m_intake, m_intake.getDeployedEncoderValue());
        var elevatorStowCommand = new ElevatorStowCommand(m_elevator, m_elevator.getStowedEncoderValue());
        // var shooterSetAngleCommand = new ShooterSetAngleCommand(m_shooter, 7.5);
        var shootNoteCommand = new ShootNoteCommand(m_intake, m_shooter, .10);

        return Commands.sequence(
            // raise elevator pivot
            elevatorDeployCommand,
            // lower intake
            intakeDeployCommand,
            Commands.parallel(
                // pivot elevator down
                elevatorStowCommand
                // raise shooter
                // shooterSetAngleCommand
                // m_shooter.commandSetAngle(7.5)
            ),
            // shoot
            shootNoteCommand
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
        var shootSpeakerNoteCommand = new ShootSpeakerCommand(m_shooterLimelight, m_intakeLimelight, m_pathPlanner, m_intake, m_shooter);

        return Commands.sequence(
            Commands.print("Executing cycle for note " + noteNumber + "..."),
            moveToNoteCommand,
            // m_pathPlanner.moveToOurNote2(),

            // m_pathPlanner.moveToRedTest1(),
            // m_pathPlanner.moveToRedTest2(),
            // m_pathPlanner.moveToRedTest3(),
            // m_pathPlanner.moveToRedTest4()

            Commands.print("Acquire note..."),
            acquireNoteCommand,

            Commands.print("Move to speaker and shoot..."),
            shootSpeakerNoteCommand
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
