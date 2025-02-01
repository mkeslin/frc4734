package frc.robot.Commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.Constants.SideToSideConstants.SideToSidePosition;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.CoralSim.CoralSimLocation;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.SideToSide;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class RobotCommands {
    public static ScoreLevel lastScoreLevel = ScoreLevel.None;
    public static ScoreSide lastScoreSide = ScoreSide.None;

    public static Command prepareCoralScoreCommand(
            ScoreLevel level,
            ScoreSide side,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        SideToSidePosition sideToSidePosition;

        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
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
            default -> {
                throw new IllegalArgumentException("Invalid ScoreSide");
            }
        }

        return Commands.runOnce(() -> {
            lastScoreLevel = level;
            lastScoreSide = side;
        }).andThen(
                Commands.parallel(
                        Commands
                                .waitSeconds(0.0)
                                .andThen(elevator.moveToSetPositionCommand(() -> elevatorPosition).asProxy()),
                        Commands
                                .waitSeconds(0.6)
                                // .andThen(arm.moveToSetPositionCommand(() -> armPosition).asProxy()),
                                .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()),
                        Commands
                                .waitSeconds(0.9)
                                .andThen(sideToSide.moveToSetPositionCommand(() -> sideToSidePosition).asProxy())));
    }

    public static Command autoPrepareCoralScoreCommand(
            ScoreLevel level,
            ScoreSide side,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        SideToSidePosition sideToSidePosition;

        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
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
            default -> {
                throw new IllegalArgumentException("Invalid ScoreSide");
            }
        }

        return Commands.runOnce(() -> {
            lastScoreLevel = level;
            lastScoreSide = side;
        }).andThen(
                Commands.parallel(
                        Commands.waitSeconds(0.5)
                                .andThen(arm.moveToSetPositionCommand(() -> armPosition)).asProxy(),
                        Commands.waitSeconds(0)
                                .andThen(sideToSide.moveToSetPositionCommand(() -> sideToSidePosition)).asProxy(),
                        Commands.waitSeconds(0)
                                .andThen(elevator.moveToSetPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command scoreCoralCommand(
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            CoralSim coralSim) {
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(ScoreLevel.L1,
                        Commands.parallel(
                        // drivetrain.moveVoltageTimeCommand(4, 0.5),
                        // elevator.movePositionDeltaCommand(() -> ElevatorConstants.SCORING_MOVEMENT).asProxy())
                        )),
                Map.entry(ScoreLevel.L2,
                        Commands.parallel(
                                // arm.movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy())),
                                arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy())),
                Map.entry(ScoreLevel.L3,
                        Commands.parallel(
                                // arm.movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy())),
                                arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy())),
                Map.entry(ScoreLevel.L4,
                        Commands.parallel(
                                // arm.movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy(),
                                // Commands.waitSeconds(0.5)
                                // .andThen(
                                // elevator.movePositionDeltaCommand(
                                // () -> ElevatorConstants.SCORING_MOVEMENT))
                                // .asProxy())),
                                arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy())),

                Map.entry(ScoreLevel.None, Commands.none()));

        return Commands.select(commandMap, () -> lastScoreLevel);
    }

    public static Command prepareIntakeCoralCommand(
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        return Commands.parallel(
                Commands.waitSeconds(0)
                        .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                Commands.waitSeconds(0)
                        .andThen(sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy()),
                Commands.waitSeconds(1.0)
                        .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy()));
    }

    public static Command intakeCoralCommand(
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        return Commands.sequence(
                prepareIntakeCoralCommand(elevator, arm, sideToSide, coralSim),
                Commands.parallel(
                        elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                elevator.movePositionDeltaCommand(() -> 0.31).asProxy()
                        .alongWith(Commands.waitSeconds(0.1).andThen(
                                coralSim.setLocationCommand(CoralSimLocation.CLAW))),
                Commands.parallel(
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToSetPositionCommand(
                                        () -> ElevatorPosition.BOTTOM).asProxy()),
                        arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()));
    }

    public static Command intakeAndScoreCommand(
            ScoreLevel level,
            ScoreSide side,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        return Commands.sequence(
                Commands.parallel(
                        elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                autoPrepareCoralScoreCommand(level, side, elevator, arm, sideToSide, coralSim)
                        .alongWith(Commands.waitSeconds(0.1).andThen(
                                coralSim.setLocationCommand(CoralSimLocation.CLAW))));
    }

    public static Command returnToStartPositions(
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide) {
        return Commands.parallel(
                Commands
                        .waitSeconds(1.0)
                        .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy()),
                Commands
                        .waitSeconds(0.5)
                        .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                Commands
                        .waitSeconds(0.0)
                        .andThen(sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy()));
    }

    // public static Command prepareAlgaeL2RemoveCommand(Elevator elevator, Arm arm) {
    // return Commands.sequence(Commands.parallel(
    // elevator.moveToSetPositionCommand(() -> ElevatorPosition.ALGAE_L2).asProxy(),
    // arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    // }

    // public static Command prepareAlgaeL3RemoveCommand(Elevator elevator, Arm arm) {
    // return Commands.sequence(Commands.parallel(
    // elevator.moveToSetPositionCommand(() -> ElevatorPosition.ALGAE_L3).asProxy(),
    // arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    // }

    // public static Command algaeRemoveCommand(Drivetrain drivetrain, Elevator
    // elevator, Arm arm) {
    // return Commands.sequence(
    // Commands.parallel(
    // drivetrain.moveVoltageTimeCommand(-2, 0.5),
    // elevator.movePositionDeltaCommand(() -> -0.06).asProxy()));
    // }
}
