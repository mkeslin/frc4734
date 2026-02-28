package frc.robot.Auto.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotState;
import frc.robot.Subsystems.Climber;

/**
 * Run-while-held climb command: runs a rotation-based ascend or descend cycle and only ends when cancelled (e.g. button released).
 * v1 uses only the climber motor; jaws stay at default/open.
 *
 * <p>Ascend: for levels 1–3, extend to level target then retract to start; transitions based on climber motor rotations.
 * After level 3 completes, holds position until released.
 *
 * <p>Descend (v1): single phase – drive climber to {@link ClimberConstants#DESCEND_TARGET_ROTATIONS}, then hold until released.
 */
public class ClimbWhileHeldCommand extends Command {

    private final Climber m_climber;
    private final boolean m_ascend;
    private final boolean m_runToCompletion;
    private final int m_maxLevels;

    private enum AscendState {
        Extending,
        Retracting,
        Done
    }

    private enum DescendState {
        RetractingToStart,
        Done
    }

    private AscendState m_ascendState = AscendState.Extending;
    private DescendState m_descendState = DescendState.RetractingToStart;
    private int m_level = 1;

    private static final double TOLERANCE = ClimberConstants.CLIMB_POSITION_TOLERANCE_ROTATIONS;

    public ClimbWhileHeldCommand(Climber climber, boolean ascend, boolean runToCompletion, int maxLevels) {
        this.m_climber = Objects.requireNonNull(climber, "climber cannot be null");
        this.m_ascend = ascend;
        this.m_runToCompletion = runToCompletion;
        this.m_maxLevels = maxLevels;
        addRequirements(climber);
    }

    /** Teleop: ascend while held (up to 3 levels), never finishes. */
    public static ClimbWhileHeldCommand ascent(Climber climber) {
        return new ClimbWhileHeldCommand(climber, true, false, 3);
    }

    /** Teleop: descend while held, never finishes. */
    public static ClimbWhileHeldCommand descent(Climber climber) {
        return new ClimbWhileHeldCommand(climber, false, false, 3);
    }

    /** Auto: one climb cycle (extend L1 → retract), then command ends. */
    public static ClimbWhileHeldCommand ascentToCompletion(Climber climber) {
        return new ClimbWhileHeldCommand(climber, true, true, 1);
    }

    @Override
    public void initialize() {
        m_ascendState = AscendState.Extending;
        m_descendState = DescendState.RetractingToStart;
        m_level = 1;
    }

    private static boolean atTarget(double current, double target) {
        return Math.abs(current - target) < TOLERANCE;
    }

    private double getExtendTarget(int level) {
        switch (level) {
            case 1: return ClimberConstants.CLIMB_L1_EXTEND_ROTATIONS;
            case 2: return ClimberConstants.CLIMB_L2_EXTEND_ROTATIONS;
            case 3: return ClimberConstants.CLIMB_L3_EXTEND_ROTATIONS;
            default: return ClimberConstants.CLIMB_L3_EXTEND_ROTATIONS;
        }
    }

    private double getStartTarget(int level) {
        switch (level) {
            case 1: return ClimberConstants.CLIMB_L1_START_ROTATIONS;
            case 2: return ClimberConstants.CLIMB_L2_START_ROTATIONS;
            case 3: return ClimberConstants.CLIMB_L3_START_ROTATIONS;
            default: return ClimberConstants.CLIMB_L3_START_ROTATIONS;
        }
    }

    @Override
    public void execute() {
        if (!RobotState.getInstance().isInitialized()) {
            m_climber.stopLift();
            return;
        }

        double position = m_climber.getPosition();

        if (m_ascend) {
            switch (m_ascendState) {
                case Extending: {
                    double target = getExtendTarget(m_level);
                    m_climber.setLiftGoalPosition(target);
                    if (atTarget(position, target)) {
                        m_ascendState = AscendState.Retracting;
                    }
                    break;
                }
                case Retracting: {
                    double target = getStartTarget(m_level);
                    m_climber.setLiftGoalPosition(target);
                    if (atTarget(position, target)) {
                        if (m_level >= m_maxLevels) {
                            m_ascendState = AscendState.Done;
                        } else {
                            m_level++;
                            m_ascendState = AscendState.Extending;
                        }
                    }
                    break;
                }
                case Done:
                    m_climber.setLiftGoalPosition(position);
                    break;
            }
        } else {
            switch (m_descendState) {
                case RetractingToStart: {
                    double target = ClimberConstants.DESCEND_TARGET_ROTATIONS;
                    m_climber.setLiftGoalPosition(target);
                    if (atTarget(position, target)) {
                        m_descendState = DescendState.Done;
                    }
                    break;
                }
                case Done:
                    m_climber.setLiftGoalPosition(position);
                    break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_runToCompletion) {
            return false;
        }
        if (m_ascend) {
            return m_ascendState == AscendState.Done;
        }
        return m_descendState == DescendState.Done;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stopLift();
    }
}
