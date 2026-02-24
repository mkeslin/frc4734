package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CANIds.SHOOTER_LEFT;
import static frc.robot.Constants.CANIds.SHOOTER_RIGHT;
import static frc.robot.Constants.CANIds.SHOOTER_CENTER;
import static edu.wpi.first.units.Units.Volts;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PositionTracker;
import frc.robot.RobotState;
import frc.robot.TelemetryCalcs;
import frc.robot.Constants.ShooterConstants.ShooterSpeed;
import frc.robot.Subsystems.Bases.BaseIntake;

/**
 * Shooter subsystem that controls the robot's shooter mechanism.
 * Uses three TalonFX motors (leader and two followers) with velocity control to shoot balls.
 * 
 * <p>Safety features:
 * <ul>
 *   <li>Prevents movement until robot is initialized via RobotState</li>
 *   <li>Extensible safety coordination hooks for future conditions</li>
 *   <li>Publishes shooter speed to NetworkTables for telemetry</li>
 * </ul>
 * 
 * @see BaseIntake
 * @see ShooterSpeed
 * @see RobotState
 */
public class Shooter extends SubsystemBase implements BaseIntake<ShooterSpeed> {
    private final DoublePublisher shooterSpeedPub = TelemetryCalcs.createMechanismsPublisher("Shooter Speed");

    private TalonFX m_shooterLeftLeaderMotor;
    private TalonFX m_shooterRightFollowerMotor;
    private TalonFX m_shooterCenterFollowerMotor;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutine;
    private final Timer m_sysIdTimer = new Timer();

    private PositionTracker m_positionTracker;

    private boolean initialized;

    public Shooter() {
        m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).div(Seconds.of(1)),
            Volts.of(0.5),
            null,
            (state) -> {
                m_sysIdTimer.reset();
                m_sysIdTimer.start();
                SignalLogger.writeString("Shooter State", state.toString());
            }),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                double v = volts.in(Volts);
                double elapsed = m_sysIdTimer.get();

                m_shooterLeftLeaderMotor.setControl(m_voltReq.withOutput(v));

                if (elapsed >= 0.2) {
                    m_shooterRightFollowerMotor.setControl(m_voltReq.withOutput(v));
                } else {
                    m_shooterRightFollowerMotor.setControl(m_voltReq.withOutput(0));
                }

                if (elapsed >= 0.4) {
                    m_shooterCenterFollowerMotor.setControl(m_voltReq.withOutput(v));
                } else {
                    m_shooterCenterFollowerMotor.setControl(m_voltReq.withOutput(0));
                }
            },
            null,
            this,
            "Shooter"));

        m_shooterLeftLeaderMotor = new TalonFX(SHOOTER_LEFT);
        m_shooterLeftLeaderMotor.setNeutralMode(NeutralModeValue.Brake);

        m_shooterRightFollowerMotor = new TalonFX(SHOOTER_RIGHT);
        m_shooterRightFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
        m_shooterRightFollowerMotor.setControl(new Follower(m_shooterLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        m_shooterCenterFollowerMotor = new TalonFX(SHOOTER_CENTER);
        m_shooterCenterFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
        m_shooterCenterFollowerMotor.setControl(new Follower(m_shooterLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        resetSpeed();
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation if needed
    }

    /**
     * Gets whether the shooter has been initialized (speed reset).
     * 
     * @return true if initialized, false otherwise
     */
    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Updates the PositionTracker reference. Used during initialization to ensure
     * all subsystems share the same PositionTracker instance with real suppliers.
     * 
     * @param positionTracker The PositionTracker instance to use (can be null)
     */
    public void setPositionTracker(PositionTracker positionTracker) {
        m_positionTracker = positionTracker;
    }

    /**
     * Checks if the shooter can run based on safety conditions.
     * This is a placeholder method that can be extended with actual safety checks.
     * 
     * @return true if the shooter can run, false otherwise
     */
    private boolean canRun() {
        // TODO: Add safety coordination checks when conditions are determined
        // Examples:
        // - Only run when ready to shoot
        // - Coordinate with Feeder mechanism
        // - Check for ball presence before shooting
        return true;
    }

    @Override
    public double getSpeed() {
        return m_shooterLeftLeaderMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetSpeed() {
        m_shooterLeftLeaderMotor.stopMotor();
        m_shooterRightFollowerMotor.stopMotor();
        m_shooterCenterFollowerMotor.stopMotor();
        initialized = true;
    }

    private Command moveToSpeedCommand(double goalVelocity) {
        return run(() -> {
            // Safety check: prevent movement until robot is initialized
            if (RobotState.getInstance().isInitialized() && canRun()) {
                VelocityVoltage velocityOut = new VelocityVoltage(0);
                velocityOut.Slot = 0;
                m_shooterLeftLeaderMotor.setControl(velocityOut.withVelocity(goalVelocity));
                // Follower motor automatically follows leader
            } else {
                m_shooterLeftLeaderMotor.stopMotor();
                m_shooterRightFollowerMotor.stopMotor();
                m_shooterCenterFollowerMotor.stopMotor();
            }
            if (m_positionTracker != null) {
                shooterSpeedPub.set(m_positionTracker.getShooterSpeed());
            }
        })
                .withName("shooter.moveToSpeed");
    }

    @Override
    public Command moveToSetSpeedCommand(Supplier<ShooterSpeed> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get().value))
                .withTimeout(3)
                .withName("shooter.moveToSetSpeed");
    }

    @Override
    public Command moveToArbitrarySpeedCommand(Supplier<Double> goalSpeedSupplier) {
        Objects.requireNonNull(goalSpeedSupplier, "goalSpeedSupplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(goalSpeedSupplier.get()))
                .withName("shooter.moveToArbitrarySpeed");
    }

    @Override
    public Command moveSpeedDeltaCommand(Supplier<Double> delta) {
        Objects.requireNonNull(delta, "delta supplier cannot be null");
        return Commands.sequence(
                moveToSpeedCommand(getSpeed() + delta.get()))
                .withName("shooter.moveSpeedDelta");
    }

    @Override
    public Command resetSpeedCommand() {
        return runOnce(this::resetSpeed).withName("shooter.resetSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            m_shooterLeftLeaderMotor.stopMotor();
            m_shooterRightFollowerMotor.stopMotor();
            m_shooterCenterFollowerMotor.stopMotor();
        })
                .andThen(() -> {
                    m_shooterLeftLeaderMotor.setNeutralMode(NeutralModeValue.Coast);
                    m_shooterRightFollowerMotor.setNeutralMode(NeutralModeValue.Coast);
                    m_shooterCenterFollowerMotor.setNeutralMode(NeutralModeValue.Coast);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooter.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
    }

    /**
     * Cleans up resources when the robot is disabled.
     * Stops all motors and closes NetworkTables publishers.
     */
    public void cleanup() {
        if (m_shooterLeftLeaderMotor != null) {
            m_shooterLeftLeaderMotor.stopMotor();
        }
        if (m_shooterRightFollowerMotor != null) {
            m_shooterRightFollowerMotor.stopMotor();
        }
        if (m_shooterCenterFollowerMotor != null) {
            m_shooterCenterFollowerMotor.stopMotor();
        }
        if (shooterSpeedPub != null) {
            shooterSpeedPub.close();
        }
    }
}
