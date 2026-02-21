package frc.robot.Controllers;

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
 * Factory for molecule commands used in competition bindings: intake (intake+floor+feeder),
 * reverse intake, shoot (shooter then delayed feeder then delayed floor), reverse shoot.
 * All commands run while held and reset subsystems on release.
 */
public final class MechanismMolecules {
    private MechanismMolecules() {}

    /**
     * Intake molecule: runs intake, floor, and feeder forward. On end, resets all three.
     */
    public static Command intakeMolecule(DeployableIntake intake, Floor floor, Feeder feeder) {
        return new ParallelCommandGroup(
                intake.moveToArbitraryIntakeSpeedCommand(() -> IntakeSpeed.IN.value),
                floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value),
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value))
                .finallyDo(interrupted -> {
                    intake.resetIntakeSpeed();
                    floor.resetSpeed();
                    feeder.resetSpeed();
                })
                .withName("MechanismMolecules.intake");
    }

    /**
     * Reverse intake molecule: runs intake, floor, and feeder reverse. On end, resets all three.
     */
    public static Command reverseIntakeMolecule(DeployableIntake intake, Floor floor, Feeder feeder) {
        return new ParallelCommandGroup(
                intake.moveToArbitraryIntakeSpeedCommand(() -> IntakeSpeed.OUT.value),
                floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.REVERSE.value),
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value))
                .finallyDo(interrupted -> {
                    intake.resetIntakeSpeed();
                    floor.resetSpeed();
                    feeder.resetSpeed();
                })
                .withName("MechanismMolecules.reverseIntake");
    }

    /**
     * Shoot molecule: backs feeder off (reverse) for SHOOT_FEEDER_BACKOFF so the ball clears the shooter
     * wheels, then starts shooter and runs feeder/floor after their delays. On end, stops shooter, feeder, and floor.
     */
    public static Command shootMolecule(Shooter shooter, Feeder feeder, Floor floor) {
        Command backoffThenShoot = Commands.sequence(
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value)
                        .withTimeout(SHOOT_FEEDER_BACKOFF),
                Commands.runOnce(feeder::resetSpeed, feeder),
                new ParallelCommandGroup(
                        shooter.moveToArbitrarySpeedCommand(() -> ShooterSpeed.FORWARD.value),
                        Commands.waitSeconds(SHOOT_FEEDER_DELAY)
                                .andThen(feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value)),
                        Commands.waitSeconds(SHOOT_FLOOR_DELAY)
                                .andThen(floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value))));
        return backoffThenShoot
                .finallyDo(interrupted -> {
                    shooter.resetSpeed();
                    feeder.resetSpeed();
                    floor.resetSpeed();
                })
                .withName("MechanismMolecules.shoot");
    }

    /**
     * Reverse shoot molecule: runs shooter, feeder, and floor reverse. On end, resets all three.
     */
    public static Command reverseShootMolecule(Shooter shooter, Feeder feeder, Floor floor) {
        return new ParallelCommandGroup(
                shooter.moveToArbitrarySpeedCommand(() -> ShooterSpeed.REVERSE.value),
                feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value),
                floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.REVERSE.value))
                .finallyDo(interrupted -> {
                    shooter.resetSpeed();
                    feeder.resetSpeed();
                    floor.resetSpeed();
                })
                .withName("MechanismMolecules.reverseShoot");
    }
}
