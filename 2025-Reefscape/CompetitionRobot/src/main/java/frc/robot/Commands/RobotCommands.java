package frc.robot.Commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PositionTracker;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Constants.SideToSideConstants.SideToSidePosition;
import frc.robot.State.StateMachine;
import frc.robot.State.StateMachineStateName;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.VisionCamera;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class RobotCommands {
    private static ScoreLevel lastScoreLevel = ScoreLevel.None;

    public static Command prepareScoreCoralCommand(
            PositionTracker positionTracker,
            ScoreLevel level,
            ScoreSide side,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            VisionCamera reefCamera) {
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
                                .waitSeconds(0.0)
                                .andThen(sideToSide.moveToSetPositionCommand(() -> sideToSidePosition)
                                        .asProxy()),
                        Commands
                                .waitSeconds(0.0)
                                .andThen(arm.moveToSetPositionCommand(() -> armPosition).asProxy()),
                        Commands
                                .waitSeconds(0.0)
                                .andThen(elevator.moveToSetPositionCommand(() -> elevatorPosition).asProxy())
                // Commands
                // .waitSeconds(0.0)
                // .andThen(arm.moveToSetPositionCommand(() -> armPosition).asProxy())
                // .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()),
                )
                        .onlyIf(() -> StateMachine.CanTransition(positionTracker, StateMachineStateName.PrepareScore))
                        .andThen(() -> lights.setSolidColor(StateMachine.GetCurrentState().Color))
        //
        );
    }

    public static Command prepareScoreCoralRetryCommand(
            PositionTracker positionTracker,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            VisionCamera reefCamera) {
                return arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()
                        .onlyIf(() -> StateMachine.CanTransition(positionTracker, StateMachineStateName.PrepareScore))
                        .andThen(() -> lights.setSolidColor(StateMachine.GetCurrentState().Color));
    }

    public static Command scoreCoralCommand(
            PositionTracker positionTracker,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            Lights lights) {
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(ScoreLevel.L1, Commands.none()),
                Map.entry(ScoreLevel.L2, arm.moveToSetPositionCommand(() -> ArmPosition.L3_SCORE).asProxy()),
                Map.entry(ScoreLevel.L3, arm.moveToSetPositionCommand(() -> ArmPosition.L3_SCORE).asProxy()),
                Map.entry(ScoreLevel.L4, arm.moveToSetPositionCommand(() -> ArmPosition.L4_SCORE).asProxy()),
                Map.entry(ScoreLevel.None, Commands.none()));

        return Commands.select(commandMap, () -> lastScoreLevel)
                .onlyIf(() -> StateMachine.CanTransition(positionTracker, StateMachineStateName.Score))
                .andThen(() -> lights.setSolidColor(StateMachine.GetCurrentState().Color));
    }

    public static Command preIntakeCoralCommand(
            PositionTracker positionTracker,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights) {
        return Commands.sequence(
                // elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy(),
                Commands.parallel(
                        Commands.waitSeconds(0.0)
                                .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                        Commands.waitSeconds(0.0)
                                .andThen(
                                        sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy()),
                        Commands.waitSeconds(0.0)
                                .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP)
                                        .asProxy())
                //
                )
        //
        ).onlyIf(() -> StateMachine.CanTransition(positionTracker, StateMachineStateName.PreIntake))
                .andThen(() -> lights.setSolidColor(StateMachine.GetCurrentState().Color));
    }

    public static Command intakeCoralCommand(
            PositionTracker positionTracker,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights) {
        return Commands.parallel(
                Commands
                        .waitSeconds(0.0)
                        .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy()),
                Commands
                        .waitSeconds(0.0)
                        .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                Commands
                        .waitSeconds(0.0)
                        .andThen(sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy())
        //
        )
                .onlyIf(() -> StateMachine.CanTransition(positionTracker, StateMachineStateName.Intake))
                .andThen(() -> lights.setSolidColor(StateMachine.GetCurrentState().Color));
    }

    public static Command postIntakeCoralCommand(
            PositionTracker positionTracker,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights) {
        return Commands.parallel(
                Commands
                        .waitSeconds(0.0)
                        .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy()),
                Commands
                        .waitSeconds(0.40)
                        .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()),
                Commands
                        .waitSeconds(0.0)
                        .andThen(sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy())
        //
        )
                .onlyIf(() -> StateMachine.CanTransition(positionTracker, StateMachineStateName.PostIntake))
                .andThen(() -> lights.setSolidColor(StateMachine.GetCurrentState().Color));
    }
}
