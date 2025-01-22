package frc.robot.Commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.CoralSim.CoralSimLocation;
import frc.robot.Subsystems.Elevator;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class RobotCommands {
    public static ScoreLevel lastScore = ScoreLevel.None;

    public static Command prepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm,
            CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
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

        return Commands.runOnce(() -> {
            lastScore = level;
        }).andThen(Commands.parallel(
                arm.moveToPositionCommand(() -> armPosition).asProxy(),
                Commands.waitSeconds(0.5).andThen(
                        elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command autoPrepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm,
            CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
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

        return Commands.runOnce(() -> {
            lastScore = level;
        }).andThen(Commands.parallel(
                Commands.waitSeconds(0.5).andThen(arm.moveToPositionCommand(() -> armPosition))
                        .asProxy(),
                Commands.waitSeconds(0).andThen(
                        elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command scoreCoralCommand(CommandSwerveDrivetrain drivetrain, Elevator elevator,
            Arm arm, CoralSim coralSim) {
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(ScoreLevel.L1,
                        Commands.parallel(drivetrain.moveVoltageTimeCommand(4, 0.5),
                                elevator.movePositionDeltaCommand(
                                        () -> ElevatorConstants.SCORING_MOVEMENT).asProxy())),
                Map.entry(ScoreLevel.L2, Commands.parallel(arm
                        .movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy())),
                Map.entry(ScoreLevel.L3, Commands.parallel(arm
                        .movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy())),
                Map.entry(ScoreLevel.L4, Commands.parallel(
                        arm.movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy(),
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.movePositionDeltaCommand(
                                        () -> ElevatorConstants.SCORING_MOVEMENT))
                                .asProxy())),
                Map.entry(ScoreLevel.None, Commands.none()));

        return Commands.select(commandMap, () -> lastScore);
    }

    public static Command prepareIntakeCoralCommand(Elevator elevator, Arm arm, CoralSim coralSim) {
        return Commands.sequence(Commands.parallel(
                elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy(),
                arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()));
    }

    public static Command intakeCoralCommand(Elevator elevator, Arm arm, CoralSim coralSim) {
        return Commands
                .sequence(prepareIntakeCoralCommand(elevator, arm, coralSim), Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                        elevator.movePositionDeltaCommand(() -> 0.31).asProxy()
                                .alongWith(Commands.waitSeconds(0.1).andThen(
                                        coralSim.setLocationCommand(CoralSimLocation.CLAW))),
                        Commands.parallel(
                                Commands.waitSeconds(0.5)
                                        .andThen(elevator.moveToPositionCommand(
                                                () -> ElevatorPosition.BOTTOM).asProxy()),
                                arm.moveToPositionCommand(() -> ArmPosition.TOP).asProxy()));
    }

    public static Command intakeIntoScoreCommand(ScoreLevel level, Elevator elevator, Arm arm,
            CoralSim coralSim) {
        return Commands
                .sequence(
                        Commands.parallel(
                                elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE)
                                        .asProxy(),
                                arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                        autoPrepareCoralScoreCommand(level, elevator, arm, coralSim)
                                .alongWith(Commands.waitSeconds(0.1).andThen(
                                        coralSim.setLocationCommand(CoralSimLocation.CLAW))));
    }

    public static Command prepareAlgaeL2RemoveCommand(Elevator elevator, Arm arm) {
        return Commands.sequence(Commands.parallel(
                elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L2).asProxy(),
                arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    public static Command prepareAlgaeL3RemoveCommand(Elevator elevator, Arm arm) {
        return Commands.sequence(Commands.parallel(
                elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L3).asProxy(),
                arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    // public static Command algaeRemoveCommand(Drivetrain drivetrain, Elevator
    // elevator, Arm arm) {
    // return Commands.sequence(
    // Commands.parallel(
    // drivetrain.moveVoltageTimeCommand(-2, 0.5),
    // elevator.movePositionDeltaCommand(() -> -0.06).asProxy()));
    // }
}
