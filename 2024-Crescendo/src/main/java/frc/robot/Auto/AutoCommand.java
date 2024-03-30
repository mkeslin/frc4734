package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Commands.ElevatorDeployCommand;
//import frc.robot.Commands.ElevatorStowCommand;
import frc.robot.Commands.IntakeDeployCommand;
import frc.robot.Commands.RobotRotateCommand;
import frc.robot.Commands.SequenceCommands.AcquireNoteCommand;
import frc.robot.Commands.SequenceCommands.ShootSpeakerCommand;
import frc.robot.Commands.ShootNoteCommand;
//import frc.robot.Commands.ShooterSetAngleCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Climber;
//import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.River;
import java.util.ArrayList;
import java.util.List;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommand extends SequentialCommandGroup {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final PathPlanner m_pathPlanner;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Climber m_climber;
    private final River m_river;
    //private final Elevator m_elevator;
    private final Limelight m_intakeLimelight;
    //private final Limelight m_shooterLimelight;

    private final int[] m_noteOrder;
    private final int m_startingPosition;
    private final boolean m_isRedAlliance;

    public AutoCommand(
        CommandSwerveDrivetrain drivetrain,
        PathPlanner pathPlanner,
        Intake intake,
        Shooter shooter,
        Climber climber,
        River river,
        //Elevator elevator,
        Limelight intakeLimelight,
        //Limelight shooterLimelight,
        int[] noteOrder,
        int startingPosition,
        boolean isRedAlliance
    ) {
        m_drivetrain = drivetrain;
        m_pathPlanner = pathPlanner;
        m_intake = intake;
        m_shooter = shooter;
        m_climber = climber;
        m_river = river;
        //m_elevator = elevator;
        m_intakeLimelight = intakeLimelight;
        //m_shooterLimelight = shooterLimelight;

        m_noteOrder = noteOrder;
        m_startingPosition = startingPosition;
        m_isRedAlliance = isRedAlliance;

        addRequirements(m_pathPlanner, m_intake, m_shooter, m_climber, /*m_elevator,*/ m_intakeLimelight/*, m_shooterLimelight*/);

        // list of commands to be executed
        List<Command> commands = new ArrayList<Command>();

        // commands to shoot preloaded note
        commands.add(shootPreloadedNote());

        // commands for the specific note cycles
        for (Integer noteNumber : m_noteOrder) {
            commands.add(moveAcquireShootCycle(noteNumber));
        }

        // set commands
        addCommands(commands.toArray(new Command[0]));
    }

    private Command shootPreloadedNote() {
        //var elevatorDeployCommand = new ElevatorDeployCommand(m_elevator, 18);
        var intakeDeployCommand = new IntakeDeployCommand(m_intake, m_intake.getDeployedEncoderValue());
        //var elevatorStowCommand = new ElevatorStowCommand(m_elevator, m_elevator.getStowedEncoderValue());
        //var shooterSetAngleCommandHigh = new ShooterSetAngleCommand(m_shooter, Shooter.MAX_PIVOT_ENCODER_VAL);
        var shootNoteCommand = new ShootNoteCommand(m_intake, m_shooter, m_river, 1.0);
        //var shooterSetAngleCommandLow = new ShooterSetAngleCommand(m_shooter, Shooter.MAX_PIVOT_ENCODER_VAL);

        // double preloadedShooterAngle = 0;
        // int preloadedShootRotation = 0;
        // switch (m_startingPosition) {
        //     case 3:
        //         preloadedShooterAngle = 5.95;
        //         preloadedShootRotation = m_isRedAlliance ? 138 : 42;
        //         break;
        //     default:
        //     case 2:
                // preloadedShooterAngle = Shooter.AUTO_SPEAKER_PIVOT_ENCODER_VAL;
        //         preloadedShootRotation = m_isRedAlliance ? 180 : 0;
        //         break;
        //     case 1:
        //         preloadedShooterAngle = 5.95;
        //         preloadedShootRotation = m_isRedAlliance ? 222 : -42;
        //         break;
        // }

        // set shooter angle
        //shooterSetAngleCommandHigh.setTarget(Shooter.AUTO_SPEAKER_PIVOT_ENCODER_VAL);
        // shooterSetAngleCommandLow.setTarget(0);

        // var robotRotateCommand = new RobotRotateCommand(m_drivetrain, preloadedShootRotation);

        return Commands.sequence(
            Commands.parallel(
                // lower intake
                Commands.print("deploy intake"),
                intakeDeployCommand,
                // raise shooter
                Commands.print("raise shoot angle")
                //shooterSetAngleCommandHigh
            ),
            // shoot
            Commands.print("shoot preloaded note"),
            shootNoteCommand,
            // lower shooter
            Commands.print("lower shoot angle")
            //shooterSetAngleCommandLow
        );
    }

    private Command moveAcquireShootCycle(int noteNumber) {
        Command moveToNoteCommand;
        double m_speed = 0.8;
        switch (noteNumber) {
            default:
            case 1:
                moveToNoteCommand = m_pathPlanner.moveToOurNote1();
                m_speed = 0.5;
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

        var acquireNoteCommand = new AcquireNoteCommand(m_intakeLimelight, m_pathPlanner, m_intake, m_river, m_speed);
        var shootSpeakerNoteCommand = new ShootSpeakerCommand(/*m_shooterLimelight,*/ m_intakeLimelight, m_pathPlanner, m_intake, m_shooter, m_river);

        // debug
        // return Commands.sequence(
        //     m_pathPlanner.moveToRedTest1(),
        //     m_pathPlanner.moveToRedTest2(),
        //     m_pathPlanner.moveToRedTest3(),
        //     m_pathPlanner.moveToRedTest4()
        // );

        return Commands.sequence(
            Commands.print("Executing cycle for note " + noteNumber + "..."),
            moveToNoteCommand,
            Commands.print("Acquire note..."),
            acquireNoteCommand,

            // Commands.print("Move to speaker..."),
            // m_pathPlanner.moveToOurSpeaker(),

            m_pathPlanner.moveToOurStart1(),

            Commands.print("Move to speaker and shoot..."),
            shootSpeakerNoteCommand,
            Commands.print("...finished executing cycle for note " + noteNumber)
        );
    }
}
