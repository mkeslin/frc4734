package frc.robot.Commands.individual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FloorConstants.ConveyorSpeed;
import frc.robot.Constants.FeederConstants.FeederSpeed;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Constants.IntakeConstants.IntakeSpeed;
import frc.robot.Constants.ShooterConstants.ShooterSpeed;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;

/**
 * Factory for individual mechanism commands used in Mechanism mode (one button per mechanism).
 * Each method returns the same command currently built inline in ControllerBindingFactory.
 */
public final class IndividualMechanismCommands {

    private IndividualMechanismCommands() {}

    public static Command shooterForward(Shooter shooter) {
        return shooter.moveToSetSpeedCommand(() -> ShooterSpeed.FORWARD);
    }

    public static Command shooterReverse(Shooter shooter) {
        return shooter.moveToSetSpeedCommand(() -> ShooterSpeed.REVERSE);
    }

    public static Command resetShooter(Shooter shooter) {
        return shooter.resetSpeedCommand();
    }

    public static Command feederForward(Feeder feeder) {
        return feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value);
    }

    public static Command feederReverse(Feeder feeder) {
        return feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value);
    }

    public static Command resetFeeder(Feeder feeder) {
        return feeder.resetSpeedCommand();
    }

    public static Command floorForward(Floor floor) {
        return floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value);
    }

    public static Command floorReverse(Floor floor) {
        return floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.REVERSE.value);
    }

    public static Command resetFloor(Floor floor) {
        return floor.resetSpeedCommand();
    }

    public static Command intakeOn(DeployableIntake intake) {
        return intake.moveToArbitraryIntakeSpeedCommand(() -> IntakeSpeed.IN.value);
    }

    public static Command intakeDeploy(DeployableIntake intake) {
        return intake.moveToArbitraryDeployPositionCommand(() -> DeployPosition.DEPLOYED.value);
    }

    public static Command intakeStow(DeployableIntake intake) {
        return intake.moveToArbitraryDeployPositionCommand(() -> DeployPosition.STOWED.value);
    }

    public static Command resetIntake(DeployableIntake intake) {
        return intake.resetIntakeSpeedCommand();
    }
}
