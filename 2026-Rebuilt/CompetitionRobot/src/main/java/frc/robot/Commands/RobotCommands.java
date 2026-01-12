package frc.robot.Commands;

import static frc.robot.Constants.CommandConstants.DEFAULT_WAIT_TIME;
import static frc.robot.Constants.CommandConstants.POST_INTAKE_ARM_DELAY;

import java.util.Map;
import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Constants.SideToSideConstants.SideToSidePosition;
import frc.robot.State.StateMachineStateName;

/**
 * Factory class for creating robot commands.
 * Uses RobotContext to reduce parameter count and improve readability.
 */
public class RobotCommands {
    private static ScoreLevel lastScoreLevel = ScoreLevel.None;

    /**
     * Prepares the robot to score coral at the specified level and side.
     * 
     * @param context The robot context containing all subsystems
     * @param level The scoring level (L1, L2, L3, L4)
     * @param side The scoring side (Left, Right, Center)
     * @return Command to prepare for scoring
     * @throws NullPointerException if context is null
     * @throws IllegalArgumentException if level or side is invalid
     */
    public static Command prepareScoreCoralCommand(
            RobotContext context,
            ScoreLevel level,
            ScoreSide side) {
        Objects.requireNonNull(context, "RobotContext cannot be null");
        Objects.requireNonNull(level, "ScoreLevel cannot be null");
        Objects.requireNonNull(side, "ScoreSide cannot be null");
        
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        SideToSidePosition sideToSidePosition;

        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.TOP;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.TOP;
            }
            case L3 -> {
                // lights.setSolidColor(255, 255, 0);
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.TOP;
                // Commands.runOnce(lights.setColors(255,255,0));
            }
            case L4 -> {
                // lights.setSolidColors(255, 0, 0);
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.TOP;
                // Commands.runOnce(lights.setColors(255,0,0));
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        switch (side) {
            case Left -> {
                sideToSidePosition = SideToSidePosition.LEFT;
            }
            case Right -> {
                sideToSidePosition = SideToSidePosition.RIGHT;
            }
            case Center -> {
                sideToSidePosition = SideToSidePosition.CENTER;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreSide");
            }
        }

        return Commands.runOnce(() -> {
            lastScoreLevel = level;
        }).andThen(
                Commands.parallel(
                        Commands
                                .waitSeconds(DEFAULT_WAIT_TIME)
                                .andThen(context.sideToSide.moveToSetPositionCommand(() -> sideToSidePosition)
                                        .asProxy()),
                        Commands
                                .waitSeconds(DEFAULT_WAIT_TIME)
                                .andThen(context.arm.moveToSetPositionCommand(() -> armPosition).asProxy()),
                        Commands
                                .waitSeconds(DEFAULT_WAIT_TIME)
                                .andThen(context.elevator.moveToSetPositionCommand(() -> elevatorPosition).asProxy())
                )
                        .onlyIf(() -> context.stateMachine.canTransition(context.positionTracker, StateMachineStateName.PrepareScore))
                        .andThen(() -> context.lights.setSolidColor(context.stateMachine.getCurrentState().Color))
        );
    }

    /**
     * Retries preparing to score coral by moving the arm to top position.
     * 
     * @param context The robot context containing all subsystems
     * @return Command to retry preparing for scoring
     * @throws NullPointerException if context is null
     */
    public static Command prepareScoreCoralRetryCommand(RobotContext context) {
        Objects.requireNonNull(context, "RobotContext cannot be null");
        return context.arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()
                .onlyIf(() -> context.stateMachine.canTransition(context.positionTracker, StateMachineStateName.PrepareScore))
                .andThen(() -> context.lights.setSolidColor(context.stateMachine.getCurrentState().Color));
    }

    /**
     * Scores coral at the previously prepared level.
     * 
     * @param context The robot context containing all subsystems
     * @return Command to score coral
     * @throws NullPointerException if context is null
     */
    public static Command scoreCoralCommand(RobotContext context) {
        Objects.requireNonNull(context, "RobotContext cannot be null");
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(ScoreLevel.L1, Commands.none()),
                Map.entry(ScoreLevel.L2, context.arm.moveToSetPositionCommand(() -> ArmPosition.L3_SCORE).asProxy()),
                Map.entry(ScoreLevel.L3, context.arm.moveToSetPositionCommand(() -> ArmPosition.L3_SCORE).asProxy()),
                Map.entry(ScoreLevel.L4, context.arm.moveToSetPositionCommand(() -> ArmPosition.L4_SCORE).asProxy()),
                Map.entry(ScoreLevel.None, Commands.none()));

        return Commands.select(commandMap, () -> lastScoreLevel)
                .onlyIf(() -> context.stateMachine.canTransition(context.positionTracker, StateMachineStateName.Score))
                .andThen(() -> context.lights.setSolidColor(context.stateMachine.getCurrentState().Color));
    }

    /**
     * Prepares the robot for intake by moving mechanisms to pre-intake positions.
     * 
     * @param context The robot context containing all subsystems
     * @return Command to prepare for intake
     * @throws NullPointerException if context is null
     */
    public static Command preIntakeCoralCommand(RobotContext context) {
        Objects.requireNonNull(context, "RobotContext cannot be null");
        return Commands.sequence(
                Commands.parallel(
                        Commands.waitSeconds(DEFAULT_WAIT_TIME)
                                .andThen(context.arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                        Commands.waitSeconds(DEFAULT_WAIT_TIME)
                                .andThen(
                                        context.sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy()),
                        Commands.waitSeconds(DEFAULT_WAIT_TIME)
                                .andThen(context.elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP)
                                        .asProxy())
                )
        ).onlyIf(() -> context.stateMachine.canTransition(context.positionTracker, StateMachineStateName.PreIntake))
                .andThen(() -> context.lights.setSolidColor(context.stateMachine.getCurrentState().Color));
    }

    /**
     * Moves mechanisms to intake positions to collect coral.
     * 
     * @param context The robot context containing all subsystems
     * @return Command to intake coral
     * @throws NullPointerException if context is null
     */
    public static Command intakeCoralCommand(RobotContext context) {
        Objects.requireNonNull(context, "RobotContext cannot be null");
        return Commands.parallel(
                Commands
                        .waitSeconds(DEFAULT_WAIT_TIME)
                        .andThen(context.elevator.moveToSetPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy()),
                Commands
                        .waitSeconds(DEFAULT_WAIT_TIME)
                        .andThen(context.arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                Commands
                        .waitSeconds(DEFAULT_WAIT_TIME)
                        .andThen(context.sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy())
        )
                .onlyIf(() -> context.stateMachine.canTransition(context.positionTracker, StateMachineStateName.Intake))
                .andThen(() -> context.lights.setSolidColor(context.stateMachine.getCurrentState().Color));
    }

    /**
     * Moves mechanisms to post-intake positions after collecting coral.
     * 
     * @param context The robot context containing all subsystems
     * @return Command to complete intake sequence
     * @throws NullPointerException if context is null
     */
    public static Command postIntakeCoralCommand(RobotContext context) {
        Objects.requireNonNull(context, "RobotContext cannot be null");
        return Commands.parallel(
                Commands
                        .waitSeconds(DEFAULT_WAIT_TIME)
                        .andThen(context.elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy()),
                Commands
                        .waitSeconds(POST_INTAKE_ARM_DELAY)
                        .andThen(context.arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()),
                Commands
                        .waitSeconds(DEFAULT_WAIT_TIME)
                        .andThen(context.sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy())
        )
                .onlyIf(() -> context.stateMachine.canTransition(context.positionTracker, StateMachineStateName.PostIntake))
                .andThen(() -> context.lights.setSolidColor(context.stateMachine.getCurrentState().Color));
    }
}
