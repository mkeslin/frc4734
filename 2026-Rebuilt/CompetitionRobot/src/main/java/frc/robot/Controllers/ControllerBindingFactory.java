package frc.robot.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Auto.commands.AutoConstants;
import frc.robot.Auto.commands.CmdFollowPath;
import frc.robot.Constants.FeederConstants.FeederSpeed;
import frc.robot.Constants.FloorConstants.ConveyorSpeed;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Constants.IntakeConstants.IntakeSpeed;
import frc.robot.Constants.ShooterConstants.ShooterSpeed;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainBindings.InputProfile;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainBindings.MechanismMode;
import frc.robot.SwerveDrivetrain.SwerveDrivetrainBindings;

/**
 * Builds and applies controller bindings by profile/mode.
 * Encapsulates drive tuning-path bindings and all mechanism bindings (Mechanism Mode and Tuning Mode).
 * Drive default command and profile cycle are configured by {@link SwerveDrivetrainBindings};
 * this factory adds the tuning-path trigger and mechanism mappings.
 */
public class ControllerBindingFactory {

    private final CommandXboxController m_driveController;
    private final CommandXboxController m_mechanismController;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final Floor m_floor;
    private final DeployableIntake m_intake;

    /**
     * Creates a factory that will bind the given controllers and subsystems.
     * Subsystems may be null; their bindings are skipped when null.
     */
    public ControllerBindingFactory(
            CommandXboxController driveController,
            CommandXboxController mechanismController,
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake) {
        m_driveController = driveController;
        m_mechanismController = mechanismController;
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_feeder = feeder;
        m_floor = floor;
        m_intake = intake;
    }

    /**
     * Registers all bindings: drive tuning path (when profile is TUNING) and mechanism bindings
     * for both Mechanism Mode and Tuning Mode. Call once during robot initialization.
     */
    public void configureBindings() {
        configureDriveTuningBindings();
        configureMechanismModeBindings();
        configureMechanismTuningBindings();
    }

    /** Drive controller: A = run tuning path when profile is TUNING. */
    private void configureDriveTuningBindings() {
        m_driveController.a()
                .and(() -> SwerveDrivetrainBindings.getCurrentProfile() == InputProfile.TUNING)
                .onTrue(new CmdFollowPath(
                        AutoConstants.TUNING_PATH_NAME,
                        AutoConstants.DEFAULT_PATH_TIMEOUT,
                        m_drivetrain));
    }

    /** Mechanism controller: normal operation (shooter, feeder, floor, intake). Active when mechanism mode is MECHANISM. */
    private void configureMechanismModeBindings() {
        if (m_shooter != null) {
            m_mechanismController.rightBumper()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_shooter.moveToSetSpeedCommand(() -> ShooterSpeed.FORWARD))
                    .onFalse(m_shooter.resetSpeedCommand());
            m_mechanismController.leftBumper()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_shooter.moveToSetSpeedCommand(() -> ShooterSpeed.REVERSE))
                    .onFalse(m_shooter.resetSpeedCommand());
            m_mechanismController.back()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_shooter.resetSpeedCommand());
        }
        if (m_feeder != null) {
            m_mechanismController.a()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.FORWARD.value))
                    .onFalse(m_feeder.resetSpeedCommand());
            m_mechanismController.x()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_feeder.moveToArbitrarySpeedCommand(() -> FeederSpeed.REVERSE.value))
                    .onFalse(m_feeder.resetSpeedCommand());
            m_mechanismController.b()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_feeder.resetSpeedCommand());
        }
        if (m_floor != null) {
            m_mechanismController.y()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.FORWARD.value))
                    .onFalse(m_floor.resetSpeedCommand());
            m_mechanismController.rightTrigger()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_floor.moveToArbitrarySpeedCommand(() -> ConveyorSpeed.REVERSE.value))
                    .onFalse(m_floor.resetSpeedCommand());
        }
        if (m_intake != null) {
            m_mechanismController.leftTrigger()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(m_intake.moveToArbitraryIntakeSpeedCommand(() -> IntakeSpeed.IN.value))
                    .onFalse(m_intake.resetIntakeSpeedCommand());
            m_mechanismController.povUp()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_intake.moveToArbitraryDeployPositionCommand(() -> DeployPosition.DEPLOYED.value));
            m_mechanismController.povDown()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_intake.moveToArbitraryDeployPositionCommand(() -> DeployPosition.STOWED.value));
        }
    }

    /** Mechanism controller: SysId bindings. Active when mechanism mode is TUNING. */
    private void configureMechanismTuningBindings() {
        if (m_shooter != null) {
            m_mechanismController.a().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_shooter.sysIdQuasistaticCommand(Direction.kForward));
            m_mechanismController.b().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_shooter.sysIdQuasistaticCommand(Direction.kReverse));
            m_mechanismController.x().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_shooter.sysIdDynamicCommand(Direction.kForward));
            m_mechanismController.y().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_shooter.sysIdDynamicCommand(Direction.kReverse));
        }
        if (m_feeder != null) {
            m_mechanismController.rightBumper().and(m_mechanismController.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_feeder.sysIdQuasistaticCommand(Direction.kForward));
            m_mechanismController.rightBumper().and(m_mechanismController.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_feeder.sysIdQuasistaticCommand(Direction.kReverse));
            m_mechanismController.rightBumper().and(m_mechanismController.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_feeder.sysIdDynamicCommand(Direction.kForward));
            m_mechanismController.rightBumper().and(m_mechanismController.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_feeder.sysIdDynamicCommand(Direction.kReverse));
        }
        if (m_floor != null) {
            m_mechanismController.leftBumper().and(m_mechanismController.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_floor.sysIdQuasistaticCommand(Direction.kForward));
            m_mechanismController.leftBumper().and(m_mechanismController.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_floor.sysIdQuasistaticCommand(Direction.kReverse));
            m_mechanismController.leftBumper().and(m_mechanismController.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_floor.sysIdDynamicCommand(Direction.kForward));
            m_mechanismController.leftBumper().and(m_mechanismController.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_floor.sysIdDynamicCommand(Direction.kReverse));
        }
        if (m_intake != null) {
            m_mechanismController.povUp().and(m_mechanismController.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdDeployQuasistaticCommand(Direction.kForward));
            m_mechanismController.povUp().and(m_mechanismController.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdDeployQuasistaticCommand(Direction.kReverse));
            m_mechanismController.povUp().and(m_mechanismController.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdDeployDynamicCommand(Direction.kForward));
            m_mechanismController.povUp().and(m_mechanismController.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdDeployDynamicCommand(Direction.kReverse));
            m_mechanismController.povDown().and(m_mechanismController.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdIntakeQuasistaticCommand(Direction.kForward));
            m_mechanismController.povDown().and(m_mechanismController.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdIntakeQuasistaticCommand(Direction.kReverse));
            m_mechanismController.povDown().and(m_mechanismController.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdIntakeDynamicCommand(Direction.kForward));
            m_mechanismController.povDown().and(m_mechanismController.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TUNING)
                    .whileTrue(m_intake.sysIdIntakeDynamicCommand(Direction.kReverse));
        }
    }
}
