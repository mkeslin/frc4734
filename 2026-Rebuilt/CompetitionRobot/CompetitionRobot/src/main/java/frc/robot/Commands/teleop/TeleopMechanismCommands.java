package frc.robot.Commands.teleop;

import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_BACKOFF;
import static frc.robot.Constants.CommandConstants.SHOOT_FEEDER_DELAY;
import static frc.robot.Constants.CommandConstants.SHOOT_FLOOR_DELAY;
import static frc.robot.Constants.FeederConstants.FeederSpeed;
import static frc.robot.Constants.FloorConstants.ConveyorSpeed;
import static frc.robot.Constants.IntakeConstants.IntakeSpeed;
import static frc.robot.Constants.ShooterConstants.ShooterSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;

/**
 * Teleop mechanism combo commands (Teleop mode). Run while held; reset on release.
 */
public final class TeleopMechanismCommands {
    private TeleopMechanismCommands() {}

    public static Command intake(DeployableIntake intake, Floor floor, Feeder feeder) {
        return new ParallelCommandGroup(
                intake.moveToArbitraryIntakeSpeedCommand(() -> IntakeSpeed.IN.value),
                floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value))
                .finallyDo(interrupted -> {
                    intake.resetIntakeSpeed();
                    floor.resetSpeed();
                })
                .withName("TeleopMechanismCommands.intake");
    }

    public static Command reverseIntake(DeployableIntake intake, Floor floor, Feeder feeder) {
        return new ParallelCommandGroup(
                intake.moveToArbitraryIntakeSpeedCommand(() -> IntakeSpeed.OUT.value),
                floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.REVERSE.value),
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value))
                .finallyDo(interrupted -> {
                    intake.resetIntakeSpeed();
                    floor.resetSpeed();
                    feeder.resetSpeed();
                })
                .withName("TeleopMechanismCommands.reverseIntake");
    }

    public static Command shoot(Shooter shooter, Feeder feeder, Floor floor) {
        Command backoff = feeder
                .moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value)
                .withTimeout(SHOOT_FEEDER_BACKOFF)
                .finallyDo(interrupted -> feeder.resetSpeed());
        Command shootPhase = new ParallelCommandGroup(
                shooter.moveToArbitrarySpeedCommand(() -> ShooterSpeed.FORWARD.value),
                Commands.waitSeconds(SHOOT_FEEDER_DELAY)
                        .andThen(feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value)),
                Commands.waitSeconds(SHOOT_FLOOR_DELAY)
                        .andThen(floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value)));
        return Commands.sequence(backoff, shootPhase)
                .finallyDo(interrupted -> {
                    shooter.resetSpeed();
                    feeder.resetSpeed();
                    floor.resetSpeed();
                })
                .withName("TeleopMechanismCommands.shoot");
    }

    public static Command reverseShoot(Shooter shooter, Feeder feeder, Floor floor) {
        return new ParallelCommandGroup(
                shooter.moveToArbitrarySpeedCommand(() -> ShooterSpeed.REVERSE.value),
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value),
                floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.REVERSE.value))
                .finallyDo(interrupted -> {
                    shooter.resetSpeed();
                    feeder.resetSpeed();
                    floor.resetSpeed();
                })
                .withName("TeleopMechanismCommands.reverseShoot");
    }
}
