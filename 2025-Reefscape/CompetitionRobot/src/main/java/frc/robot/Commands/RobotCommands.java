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
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CoralSim;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class RobotCommands {
    public static ScoreLevel lastScoreLevel = ScoreLevel.None;
    public static ScoreSide lastScoreSide = ScoreSide.None;

    public static Command moveIntermediatePrepareScoreCoralCommand(
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            CoralSim coralSim) {
        return Commands.parallel(
                // Commands.runOnce(() -> lights.setColors(0, 255, 0)).asProxy(),
                Commands.waitSeconds(0.35)
                        .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()),
                // Commands.waitSeconds(0)
                // .andThen(sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy()),
                Commands.waitSeconds(0.0)
                        .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.L3).asProxy()));
    }

    public static Command prepareScoreCoralCommand(
            ScoreLevel level,
            ScoreSide side,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            CoralSim coralSim) {
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
                lights.setSolidColors(255, 255, 0);
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.TOP;
                // Commands.runOnce(lights.setColors(255,255,0));
            }
            case L4 -> {
                lights.setSolidColors(255, 0, 0);
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
                                .waitSeconds(0.35)
                                .andThen(sideToSide.moveToSetPositionCommand(() -> sideToSidePosition)
                                        .asProxy()),
                        Commands
                                .waitSeconds(0.35)
                                .andThen(arm.moveToSetPositionCommand(() -> armPosition).asProxy()),
                        Commands
                                .waitSeconds(0.0)
                                .andThen(elevator.moveToSetPositionCommand(() -> elevatorPosition).asProxy())
                // Commands
                // .waitSeconds(0.0)
                // .andThen(arm.moveToSetPositionCommand(() -> armPosition).asProxy())
                // .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()),
                ));
    }

    public static Command prepareScoreCoralAndCenterToReefCommand(
            ScoreLevel scoreLevel,
            ScoreSide scoreSide,
            boolean centerToReef,
            CenterToReefCommand centerToReefCommand,
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            Lights lights,
            Limelight reefLimelight,
            CoralSim coralSim) {
        return Commands.sequence(
                RobotCommands.prepareScoreCoralCommand(scoreLevel, scoreSide, drivetrain, elevator, arm,
                        sideToSide, lights, reefLimelight, coralSim),
                centerToReefCommand.unless(() -> !centerToReef)
        //
        );
    }

    public static Command scoreCoralCommand(
            CommandSwerveDrivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            Lights lights,
            CoralSim coralSim) {
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(ScoreLevel.L1,
                        Commands.parallel(
                                // drivetrain.moveVoltageTimeCommand(4, 0.5),
                                // elevator.movePositionDeltaCommand(() ->
                                // ElevatorConstants.SCORING_MOVEMENT).asProxy())
                                Commands.runOnce(() -> lights.setSolidColors(0, 0, 64)).asProxy())),
                Map.entry(ScoreLevel.L2,
                        Commands.parallel(
                                // arm.movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy())),
                                Commands.runOnce(() -> lights.setSolidColors(0, 0, 128)).asProxy(),
                                arm.moveToSetPositionCommand(() -> ArmPosition.L3_SCORE).asProxy())),
                Map.entry(ScoreLevel.L3,
                        Commands.parallel(
                                // arm.movePositionDeltaCommand(() -> ArmConstants.SCORING_MOVEMENT).asProxy())),
                                Commands.runOnce(() -> lights.setSolidColors(0, 0, 191)).asProxy(),
                                arm.moveToSetPositionCommand(() -> ArmPosition.L3_SCORE).asProxy())),
                Map.entry(ScoreLevel.L4,
                        Commands.sequence(
                                // Commands.run(() -> drivetrain.setRelativeSpeed(0.5, 0, 0))
                                // .withTimeout(0.35)
                                // .andThen(Commands.runOnce(() -> drivetrain.setRelativeSpeed(0, 0, 0)))
                                // .asProxy(),
                                Commands.runOnce(() -> lights.setSolidColors(0, 0, 255)).asProxy(),
                                arm.moveToSetPositionCommand(() -> ArmPosition.L4_SCORE).asProxy()
                        // Commands.run(() -> drivetrain.setRelativeSpeed(-0.5, 0, 0)).asProxy().withTimeout(0.45)
                        // .andThen(Commands.runOnce(() -> drivetrain.setRelativeSpeed(0, 0, 0)))
                        )),

                Map.entry(ScoreLevel.None, Commands.none()));

        return Commands.select(commandMap, () -> lastScoreLevel);
    }

    public static Command prepareIntakeCoralCommand(
            PositionTracker positionTracker,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide,
            CoralSim coralSim) {
        return Commands.sequence(
                // elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy(),
                Commands.parallel(
                        Commands.waitSeconds(0.5)
                                .andThen(arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                        Commands.waitSeconds(0)
                                .andThen(
                                        sideToSide.moveToSetPositionCommand(() -> SideToSidePosition.CENTER).asProxy()),
                        Commands.waitSeconds(0)
                                .andThen(elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE_PREP)
                                        .asProxy())
                //
                )
        //
        ).unless(() -> positionTracker.getCoralInArm());
    }

    // public static Command intakeCoralCommand(
    // Elevator elevator,
    // Arm arm,
    // SideToSide sideToSide,
    // Lights lights,
    // CoralSim coralSim) {
    // return Commands.sequence(
    // prepareIntakeCoralCommand(elevator, arm, sideToSide, coralSim),
    // Commands.parallel(
    // Commands.runOnce(() -> lights.setSolidColors(255, 0, 255)).asProxy(),
    // elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
    // arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
    // elevator.movePositionDeltaCommand(() -> 0.31).asProxy()
    // .alongWith(Commands.waitSeconds(0.1).andThen(
    // coralSim.setLocationCommand(CoralSimLocation.CLAW))),
    // Commands.parallel(
    // Commands.waitSeconds(0.5)
    // .andThen(elevator.moveToSetPositionCommand(
    // () -> ElevatorPosition.BOTTOM).asProxy()),
    // arm.moveToSetPositionCommand(() -> ArmPosition.TOP).asProxy()));
    // }

    // public static Command intakeAndScoreCommand(
    // ScoreLevel level,
    // ScoreSide side,
    // Elevator elevator,
    // Arm arm,
    // SideToSide sideToSide,
    // Lights lights,
    // CoralSim coralSim) {
    // return Commands.sequence(
    // Commands.parallel(
    // elevator.moveToSetPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
    // arm.moveToSetPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
    // autoPrepareCoralScoreCommand(level, side, elevator, arm, sideToSide, lights, coralSim)
    // .alongWith(Commands.waitSeconds(0.1).andThen(
    // coralSim.setLocationCommand(CoralSimLocation.CLAW))));
    // }

    public static Command returnToStartPositions(
            PositionTracker positionTracker,
            Elevator elevator,
            Arm arm,
            SideToSide sideToSide) {
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
        ).unless(() -> positionTracker.getCoralInArm());
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
