package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.RobotCommands;
import frc.robot.Commands.RobotContext;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Subsystems.Climber;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Configures all controller bindings for the robot.
 * Consolidates mechanism, drive, and arcade controller bindings into a single class.
 */
public class BindingConfigurator {
    private final CommandXboxController m_driveController;
    private final CommandXboxController m_mechanismController;
    private final CommandXboxController m_arcadeController;
    private final RobotContext m_robotContext;
    private final PositionTracker m_positionTracker;
    private final Climber m_climber;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CenterToReefCommand m_centerToReefCommand;

    /**
     * Creates a new BindingConfigurator with all required dependencies.
     * 
     * @param driveController The drive controller
     * @param mechanismController The mechanism controller
     * @param arcadeController The arcade controller
     * @param robotContext The robot context containing all subsystems
     * @param climber The climber subsystem
     * @param centerToReefCommand The center to reef command
     */
    public BindingConfigurator(
            CommandXboxController driveController,
            CommandXboxController mechanismController,
            CommandXboxController arcadeController,
            RobotContext robotContext,
            Climber climber,
            CenterToReefCommand centerToReefCommand) {
        m_driveController = driveController;
        m_mechanismController = mechanismController;
        m_arcadeController = arcadeController;
        m_robotContext = robotContext;
        m_positionTracker = robotContext.positionTracker;
        m_climber = climber;
        m_drivetrain = robotContext.drivetrain;
        m_centerToReefCommand = centerToReefCommand;
    }

    /**
     * Configures all controller bindings.
     */
    public void configureAllBindings() {
        configureMechanismBindings();
        configureDriveBindings();
        configureArcadeBindings();
    }

    /**
     * Configures bindings for the mechanism controller.
     */
    public void configureMechanismBindings() {
        // GO TO PRE-INTAKE
        m_mechanismController.a()
                .onTrue(RobotCommands.preIntakeCoralCommand(m_robotContext));

        // INTAKE & POST-INTAKE
        m_mechanismController.b().onTrue(
                Commands.sequence(
                        RobotCommands.intakeCoralCommand(m_robotContext),
                        RobotCommands.postIntakeCoralCommand(m_robotContext)
                ));

        // RESET POSE
        m_mechanismController.start()
                .onTrue(RobotCommands.intakeCoralCommand(m_robotContext));
    }

    /**
     * Configures bindings for the drive controller.
     */
    public void configureDriveBindings() {
        m_driveController.leftTrigger().onTrue(m_centerToReefCommand);

        // CLIMBER
        m_driveController.y().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.ACQUIRE));
        m_driveController.x().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.CLIMB));

        m_driveController.povRight().whileTrue(Commands.run(() -> m_climber.setVoltage(-1.75)));
        m_driveController.povRight().onFalse(Commands.run(() -> m_climber.setVoltage(0.0)));

        m_driveController.povLeft().whileTrue(Commands.run(() -> m_climber.setVoltage(-3.5)));
        m_driveController.povLeft().onFalse(Commands.run(() -> m_climber.setVoltage(0.0)));
    }

    /**
     * Configures bindings for the arcade controller.
     */
    public void configureArcadeBindings() {
        var centerToReef = true;

        m_arcadeController.rightTrigger().onTrue(Commands.sequence(
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L2, ScoreSide.Left, centerToReef),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                        .withTimeout(0.15)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy()
        ));
        m_arcadeController.b()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Left, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.12)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                ));
        m_arcadeController.a()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L4, ScoreSide.Left, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.17)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                ));
        m_arcadeController.x()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L4, ScoreSide.Right, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.15)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                ));
        m_arcadeController.y()
                .onTrue(Commands.sequence(
                        prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Right, centerToReef),
                        Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                                .withTimeout(0.12)
                                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                                .asProxy()
                ));
        m_arcadeController.rightBumper().onTrue(Commands.sequence(
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L2, ScoreSide.Right, centerToReef),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(-0.5, 0, 0))
                        .withTimeout(0.17)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy()
        ));

        m_arcadeController.start().onTrue(
                prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Center, centerToReef));

        var scoreCommand = Commands.sequence(
                // move arm down
                RobotCommands.scoreCoralCommand(m_robotContext),
                // move forward to set coral if not completely placed
                Commands.run(() -> m_drivetrain.setRelativeSpeed(0.75, 0, 0)).withTimeout(0.12)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy(),
                Commands.run(() -> m_drivetrain.setRelativeSpeed(-1.0, 0, 0)).withTimeout(0.35)
                        .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(0, 0, 0)))
                        .asProxy(),
                RobotCommands.preIntakeCoralCommand(m_robotContext)
                        .onlyIf(() -> !m_positionTracker.getCoralInArm()),
                RobotCommands.prepareScoreCoralRetryCommand(m_robotContext)
                        .onlyIf(() -> m_positionTracker.getCoralInArm())
        );
        m_arcadeController.leftBumper().onTrue(scoreCommand);
        m_arcadeController.leftTrigger().onTrue(scoreCommand);
    }

    private Command prepareScoreCoralAndCenterToReefCommand(ScoreLevel scoreLevel, ScoreSide scoreSide,
            boolean centerToReef) {
        var centerToReefCommand = NamedCommands.getCommand("centerToReefCommand");
        return Commands.parallel(
                Commands
                        .waitSeconds(0.0)
                        .andThen(RobotCommands.prepareScoreCoralCommand(m_robotContext, scoreLevel, scoreSide)),
                Commands
                        .waitSeconds(0.0)
                        .andThen(centerToReefCommand.unless(() -> !centerToReef))
        );
    }
}
