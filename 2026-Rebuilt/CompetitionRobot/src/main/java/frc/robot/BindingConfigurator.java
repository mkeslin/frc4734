package frc.robot;

import static frc.robot.Constants.CommandConstants.APPROACH_SCORE_SPEED;
import static frc.robot.Constants.CommandConstants.CLIMBER_FAST_VOLTAGE;
import static frc.robot.Constants.CommandConstants.CLIMBER_SLOW_VOLTAGE;
import static frc.robot.Constants.CommandConstants.CLIMBER_STOP_VOLTAGE;
import static frc.robot.Constants.CommandConstants.DEFAULT_WAIT_TIME;
import static frc.robot.Constants.CommandConstants.LONG_DRIVE_TIMEOUT;
import static frc.robot.Constants.CommandConstants.MEDIUM_DRIVE_TIMEOUT;
import static frc.robot.Constants.CommandConstants.PLACE_CORAL_FORWARD_SPEED;
import static frc.robot.Constants.CommandConstants.POST_SCORE_BACKWARD_SPEED;
import static frc.robot.Constants.CommandConstants.POST_SCORE_BACKWARD_TIMEOUT;
import static frc.robot.Constants.CommandConstants.SHORT_DRIVE_TIMEOUT;
import static frc.robot.Constants.CommandConstants.STOP_SPEED;

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
        // Removed for 2026 - commands use deleted subsystems (Elevator, Arm, SideToSide)
        // GO TO PRE-INTAKE
        // m_mechanismController.a()
        //         .onTrue(RobotCommands.preIntakeCoralCommand(m_robotContext));

        // INTAKE & POST-INTAKE
        // m_mechanismController.b().onTrue(
        //         Commands.sequence(
        //                 RobotCommands.intakeCoralCommand(m_robotContext),
        //                 RobotCommands.postIntakeCoralCommand(m_robotContext)
        //         ));

        // RESET POSE
        // m_mechanismController.start()
        //         .onTrue(RobotCommands.intakeCoralCommand(m_robotContext));
    }

    /**
     * Configures bindings for the drive controller.
     */
    public void configureDriveBindings() {
        m_driveController.leftTrigger().onTrue(m_centerToReefCommand);

        // CLIMBER
        m_driveController.y().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.ACQUIRE));
        m_driveController.x().onTrue(m_climber.moveToSetPositionCommand(() -> ClimberPosition.CLIMB));

        m_driveController.povRight().whileTrue(Commands.run(() -> m_climber.setVoltage(CLIMBER_SLOW_VOLTAGE)));
        m_driveController.povRight().onFalse(Commands.run(() -> m_climber.setVoltage(CLIMBER_STOP_VOLTAGE)));

        m_driveController.povLeft().whileTrue(Commands.run(() -> m_climber.setVoltage(CLIMBER_FAST_VOLTAGE)));
        m_driveController.povLeft().onFalse(Commands.run(() -> m_climber.setVoltage(CLIMBER_STOP_VOLTAGE)));
    }

    /**
     * Configures bindings for the arcade controller.
     * 
     * NOTE: Most bindings commented out for 2026 - commands use deleted subsystems (Elevator, Arm, SideToSide)
     */
    public void configureArcadeBindings() {
        // Removed for 2026 - all scoring commands use deleted subsystems
        // final boolean centerToReef = true;

        // Scoring bindings with approach movement
        // m_arcadeController.rightTrigger().onTrue(
        //         createScoreAndApproachCommand(ScoreLevel.L2, ScoreSide.Left, centerToReef, MEDIUM_DRIVE_TIMEOUT));
        
        // m_arcadeController.b().onTrue(
        //         createScoreAndApproachCommand(ScoreLevel.L3, ScoreSide.Left, centerToReef, SHORT_DRIVE_TIMEOUT));
        
        // m_arcadeController.a().onTrue(
        //         createScoreAndApproachCommand(ScoreLevel.L4, ScoreSide.Left, centerToReef, LONG_DRIVE_TIMEOUT));
        
        // m_arcadeController.x().onTrue(
        //         createScoreAndApproachCommand(ScoreLevel.L4, ScoreSide.Right, centerToReef, MEDIUM_DRIVE_TIMEOUT));
        
        // m_arcadeController.y().onTrue(
        //         createScoreAndApproachCommand(ScoreLevel.L3, ScoreSide.Right, centerToReef, SHORT_DRIVE_TIMEOUT));
        
        // m_arcadeController.rightBumper().onTrue(
        //         createScoreAndApproachCommand(ScoreLevel.L2, ScoreSide.Right, centerToReef, LONG_DRIVE_TIMEOUT));

        // Center scoring (no approach movement)
        // m_arcadeController.start().onTrue(
        //         prepareScoreCoralAndCenterToReefCommand(ScoreLevel.L3, ScoreSide.Center, centerToReef));

        // Score command sequence
        // Removed for 2026 - uses deleted subsystems
        // Command scoreCommand = createScoreSequenceCommand();
        // m_arcadeController.leftBumper().onTrue(scoreCommand);
        // m_arcadeController.leftTrigger().onTrue(scoreCommand);
    }

    /**
     * Creates a command sequence that prepares to score, centers to reef, and approaches the scoring position.
     * 
     * REMOVED FOR 2026 - Uses prepareScoreCoralCommand which requires deleted subsystems
     * 
     * @param scoreLevel The scoring level
     * @param scoreSide The scoring side
     * @param centerToReef Whether to center to reef
     * @param approachTimeout Timeout for the approach movement
     * @return The complete command sequence
     */
    /*
    private Command createScoreAndApproachCommand(ScoreLevel scoreLevel, ScoreSide scoreSide, 
            boolean centerToReef, double approachTimeout) {
        return Commands.sequence(
                prepareScoreCoralAndCenterToReefCommand(scoreLevel, scoreSide, centerToReef),
                createApproachMovementCommand(approachTimeout)
        );
    }
    */

    /**
     * Creates a command that moves the drivetrain forward to approach the scoring position.
     * 
     * @param timeout The timeout for the movement
     * @return Command that approaches and then stops
     */
    private Command createApproachMovementCommand(double timeout) {
        return Commands.run(() -> m_drivetrain.setRelativeSpeed(APPROACH_SCORE_SPEED, STOP_SPEED, STOP_SPEED))
                .withTimeout(timeout)
                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(STOP_SPEED, STOP_SPEED, STOP_SPEED)))
                .asProxy();
    }

    /**
     * Creates the complete score sequence command.
     * Includes scoring, placing coral forward, backing away, and conditional post-intake or retry.
     * 
     * REMOVED FOR 2026 - Uses commands that require deleted subsystems
     * 
     * @return The complete score sequence command
     */
    /*
    private Command createScoreSequenceCommand() {
        return Commands.sequence(
                // Move arm down to score
                RobotCommands.scoreCoralCommand(m_robotContext),
                // Move forward to place coral if not completely placed
                createPlaceCoralForwardCommand(),
                // Back away from scoring position
                createBackAwayCommand(),
                // Conditional: pre-intake if no coral, or retry if coral still in arm
                RobotCommands.preIntakeCoralCommand(m_robotContext)
                        .onlyIf(() -> !m_positionTracker.getCoralInArm()),
                RobotCommands.prepareScoreCoralRetryCommand(m_robotContext)
                        .onlyIf(() -> m_positionTracker.getCoralInArm())
        );
    }
    */

    /**
     * Creates a command that moves forward to place coral.
     * 
     * @return Command that moves forward and stops
     */
    private Command createPlaceCoralForwardCommand() {
        return Commands.run(() -> m_drivetrain.setRelativeSpeed(PLACE_CORAL_FORWARD_SPEED, STOP_SPEED, STOP_SPEED))
                .withTimeout(SHORT_DRIVE_TIMEOUT)
                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(STOP_SPEED, STOP_SPEED, STOP_SPEED)))
                .asProxy();
    }

    /**
     * Creates a command that backs away from the scoring position.
     * 
     * @return Command that backs away and stops
     */
    private Command createBackAwayCommand() {
        return Commands.run(() -> m_drivetrain.setRelativeSpeed(POST_SCORE_BACKWARD_SPEED, STOP_SPEED, STOP_SPEED))
                .withTimeout(POST_SCORE_BACKWARD_TIMEOUT)
                .andThen(Commands.runOnce(() -> m_drivetrain.setRelativeSpeed(STOP_SPEED, STOP_SPEED, STOP_SPEED)))
                .asProxy();
    }

    /**
     * Creates a command that prepares to score coral and optionally centers to reef.
     * Both actions run in parallel with default wait times.
     * 
     * REMOVED FOR 2026 - Uses prepareScoreCoralCommand which requires deleted subsystems
     * 
     * @param scoreLevel The scoring level
     * @param scoreSide The scoring side
     * @param centerToReef Whether to center to reef
     * @return Command that prepares scoring and centers to reef in parallel
     */
    /*
    private Command prepareScoreCoralAndCenterToReefCommand(ScoreLevel scoreLevel, ScoreSide scoreSide,
            boolean centerToReef) {
        Command centerToReefCommand = NamedCommands.getCommand("centerToReefCommand");
        return Commands.parallel(
                // Removed for 2026 - prepareScoreCoralCommand uses deleted subsystems
                // Commands
                //         .waitSeconds(DEFAULT_WAIT_TIME)
                //         .andThen(RobotCommands.prepareScoreCoralCommand(m_robotContext, scoreLevel, scoreSide)),
                Commands
                        .waitSeconds(DEFAULT_WAIT_TIME)
                        .andThen(centerToReefCommand.unless(() -> !centerToReef))
        );
    }
    */
}
