package frc.robot.Controllers;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Auto.commands.AutoConstants;
import frc.robot.Auto.commands.CmdFollowPath;
import frc.robot.Commands.individual.IndividualMechanismCommands;
import frc.robot.Commands.teleop.TeleopMechanismCommands;
import frc.robot.Constants.IntakeConstants.DeployPosition;
import frc.robot.Subsystems.Climber;
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
 * Modes: Teleop (teleop commands), SysId (SysId commands), Mechanism (individual mechanism commands).
 */
public class ControllerBindingFactory {

    private final CommandXboxController m_driveController;
    private final CommandXboxController m_mechanismController;
    private final CommandXboxController m_arcadeController;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final Floor m_floor;
    private final DeployableIntake m_intake;
    private final Climber m_climber;

    /**
     * Creates a factory that will bind the given controllers and subsystems.
     * Subsystems may be null; their bindings are skipped when null.
     */
    public ControllerBindingFactory(
            CommandXboxController driveController,
            CommandXboxController mechanismController,
            CommandXboxController arcadeController,
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            Feeder feeder,
            Floor floor,
            DeployableIntake intake,
            Climber climber) {
        m_driveController = driveController;
        m_mechanismController = mechanismController;
        m_arcadeController = arcadeController;
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_feeder = feeder;
        m_floor = floor;
        m_intake = intake;
        m_climber = climber;
    }

    /**
     * Registers all bindings: drive tuning path (when profile is MECHANISM) and mechanism bindings
     * for TELEOP, SYSID, and MECHANISM modes. Call once during robot initialization.
     */
    public void configureBindings() {
        configureDriveTuningBindings();
        configureMechanismModeBindings(m_mechanismController);
        configureMechanismModeBindings(m_arcadeController);
        configureMechanismSysIdBindings(m_mechanismController);
        configureMechanismSysIdBindings(m_arcadeController);
        configureMechanismIndividualBindings(m_mechanismController);
        configureMechanismIndividualBindings(m_arcadeController);
    }

    /** Drive controller: A = run tuning path when profile is MECHANISM. */
    private void configureDriveTuningBindings() {
        m_driveController.a()
                .and(() -> SwerveDrivetrainBindings.getCurrentProfile() == InputProfile.MECHANISM)
                .onTrue(new CmdFollowPath(
                        AutoConstants.TUNING_PATH_NAME,
                        AutoConstants.DEFAULT_PATH_TIMEOUT,
                        m_drivetrain));
    }

    /** Mechanism or arcade controller: teleop bindings (intake combo, shoot combo, deploy/stow). Active when mechanism mode is TELEOP. */
    private void configureMechanismModeBindings(CommandXboxController c) {
        if (m_intake != null) {
            c.povDown()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .onTrue(m_intake.moveToArbitraryDeployPositionCommand(() -> DeployPosition.DEPLOYED.value));
            c.povUp()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .onTrue(m_intake.moveToArbitraryDeployPositionCommand(() -> DeployPosition.STOWED.value));
        }
        // Mechanism controller A/B/X/Y (TELEOP): shooter speed mode. Y=dynamic, X=115, A=127, B=140 RPS.
        c.y()
                .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                .onTrue(Commands.runOnce(TeleopMechanismCommands::setDynamicShooterSpeed));
        c.x()
                .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                .onTrue(Commands.runOnce(() -> TeleopMechanismCommands.setFixedShooterSpeed(127)));
        c.a()
                .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                .onTrue(Commands.runOnce(() -> TeleopMechanismCommands.setFixedShooterSpeed(131)));
        c.b()
                .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                .onTrue(Commands.runOnce(() -> TeleopMechanismCommands.setFixedShooterSpeed(135)));
        // Defer so each button press gets a fresh command instance; reusing a cancelled command can cause scheduler errors / e-stop
        if (m_intake != null && m_floor != null && m_feeder != null) {
            c.leftTrigger()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .whileTrue(Commands.defer(
                            () -> TeleopMechanismCommands.intake(m_intake, m_floor, m_feeder),
                            Set.<Subsystem>of(m_intake, m_floor, m_feeder)));
            c.leftBumper()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .whileTrue(Commands.defer(
                            () -> TeleopMechanismCommands.reverseIntake(m_intake, m_floor, m_feeder),
                            Set.<Subsystem>of(m_intake, m_floor, m_feeder)));
        }
        if (m_shooter != null && m_feeder != null && m_floor != null) {
            HashSet<Subsystem> shootSubsystems = new HashSet<>();
            shootSubsystems.add(m_shooter);
            shootSubsystems.add(m_feeder);
            shootSubsystems.add(m_floor);
            if (m_intake != null) {
                shootSubsystems.add(m_intake);
            }
            c.rightTrigger()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .whileTrue(Commands.defer(
                            () -> TeleopMechanismCommands.shoot(m_shooter, m_feeder, m_floor, m_drivetrain, m_intake),
                            shootSubsystems));
            c.rightBumper()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .whileTrue(Commands.defer(
                            () -> TeleopMechanismCommands.reverseShoot(m_shooter, m_feeder, m_floor),
                            Set.<Subsystem>of(m_shooter, m_feeder, m_floor)));
        }
        if (m_shooter != null || m_feeder != null || m_floor != null) {
            c.back()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .onTrue(Commands.runOnce(() -> {
                        if (m_shooter != null) m_shooter.resetSpeed();
                        if (m_feeder != null) m_feeder.resetSpeed();
                        if (m_floor != null) m_floor.resetSpeed();
                        if (m_intake != null) m_intake.resetIntakeSpeed();
                    }));
        }
        if (m_climber != null) {
            c.povRight()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.climbExtend(m_climber),
                            Set.<Subsystem>of(m_climber)));
            c.povLeft()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.TELEOP)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.climbRetract(m_climber),
                            Set.<Subsystem>of(m_climber)));
        }
    }

    /** Mechanism or arcade controller: SysId bindings (PID tuning). Active when mechanism mode is SYSID. */
    private void configureMechanismSysIdBindings(CommandXboxController c) {
        // Phoenix 6 SignalLogger: start before running the four tests, stop after (one log file for SysId tool).
        c.back()
                .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                .onTrue(Commands.runOnce(SignalLogger::start));
        c.start()
                .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                .onTrue(Commands.runOnce(SignalLogger::stop));

        if (m_shooter != null) {
            c.a().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_shooter.sysIdQuasistaticCommand(Direction.kForward));
            c.b().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_shooter.sysIdQuasistaticCommand(Direction.kReverse));
            c.x().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_shooter.sysIdDynamicCommand(Direction.kForward));
            c.y().and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_shooter.sysIdDynamicCommand(Direction.kReverse));
        }
        if (m_feeder != null) {
            c.rightBumper().and(c.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_feeder.sysIdQuasistaticCommand(Direction.kForward));
            c.rightBumper().and(c.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_feeder.sysIdQuasistaticCommand(Direction.kReverse));
            c.rightBumper().and(c.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_feeder.sysIdDynamicCommand(Direction.kForward));
            c.rightBumper().and(c.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_feeder.sysIdDynamicCommand(Direction.kReverse));
        }
        if (m_floor != null) {
            c.leftBumper().and(c.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_floor.sysIdQuasistaticCommand(Direction.kForward));
            c.leftBumper().and(c.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_floor.sysIdQuasistaticCommand(Direction.kReverse));
            c.leftBumper().and(c.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_floor.sysIdDynamicCommand(Direction.kForward));
            c.leftBumper().and(c.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_floor.sysIdDynamicCommand(Direction.kReverse));
        }
        if (m_intake != null) {
            c.povUp().and(c.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdDeployQuasistaticCommand(Direction.kForward));
            c.povUp().and(c.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdDeployQuasistaticCommand(Direction.kReverse));
            c.povUp().and(c.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdDeployDynamicCommand(Direction.kForward));
            c.povUp().and(c.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdDeployDynamicCommand(Direction.kReverse));
            c.povDown().and(c.a()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdIntakeQuasistaticCommand(Direction.kForward));
            c.povDown().and(c.b()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdIntakeQuasistaticCommand(Direction.kReverse));
            c.povDown().and(c.x()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdIntakeDynamicCommand(Direction.kForward));
            c.povDown().and(c.y()).and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.SYSID)
                    .whileTrue(m_intake.sysIdIntakeDynamicCommand(Direction.kReverse));
        }
    }

    /**
     * Mechanism or arcade controller: individual mechanism buttons. Active when mechanism mode is MECHANISM.
     * Uses defer() so each button press gets a fresh command instance; reusing a cancelled command
     * can cause scheduler errors / e-stop (same rationale as teleop combos).
     */
    private void configureMechanismIndividualBindings(CommandXboxController c) {
        if (m_shooter != null) {
            c.rightBumper()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.shooterForward(m_shooter),
                            Set.<Subsystem>of(m_shooter)))
                    .onFalse(IndividualMechanismCommands.resetShooter(m_shooter));
            c.leftBumper()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.shooterReverse(m_shooter),
                            Set.<Subsystem>of(m_shooter)))
                    .onFalse(IndividualMechanismCommands.resetShooter(m_shooter));
            c.back()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(IndividualMechanismCommands.resetShooter(m_shooter));
        }
        if (m_feeder != null) {
            c.a().and(c.start().negate())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.feederForward(m_feeder),
                            Set.<Subsystem>of(m_feeder)))
                    .onFalse(IndividualMechanismCommands.resetFeeder(m_feeder));
            c.x().and(c.start().negate())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.feederReverse(m_feeder),
                            Set.<Subsystem>of(m_feeder)))
                    .onFalse(IndividualMechanismCommands.resetFeeder(m_feeder));
            c.b().and(c.start().negate())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(IndividualMechanismCommands.resetFeeder(m_feeder));
        }
        if (m_floor != null) {
            c.y().and(c.start().negate())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.floorForward(m_floor),
                            Set.<Subsystem>of(m_floor)))
                    .onFalse(IndividualMechanismCommands.resetFloor(m_floor));
            c.rightTrigger()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.floorReverse(m_floor),
                            Set.<Subsystem>of(m_floor)))
                    .onFalse(IndividualMechanismCommands.resetFloor(m_floor));
        }
        if (m_intake != null) {
            c.leftTrigger()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.intakeOn(m_intake),
                            Set.<Subsystem>of(m_intake)))
                    .onFalse(IndividualMechanismCommands.resetIntake(m_intake));
            c.povDown()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(IndividualMechanismCommands.intakeDeploy(m_intake));
            c.povUp()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(IndividualMechanismCommands.intakeStow(m_intake));
        }
        // Climber (lift + jaws): voltage control while held, no position stops
        if (m_climber != null) {
            c.povRight()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.climbExtend(m_climber),
                            Set.<Subsystem>of(m_climber)));
            c.povLeft()
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .whileTrue(Commands.defer(
                            () -> IndividualMechanismCommands.climbRetract(m_climber),
                            Set.<Subsystem>of(m_climber)));
            c.start().and(c.a())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_climber.openLeftJawCommand());
            c.start().and(c.b())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_climber.closeLeftJawCommand());
            c.start().and(c.x())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_climber.openRightJawCommand());
            c.start().and(c.y())
                    .and(() -> SwerveDrivetrainBindings.getMechanismMode() == MechanismMode.MECHANISM)
                    .onTrue(m_climber.closeRightJawCommand());
        }
    }
}
