package frc.robot.Telemetry;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Auto.AutoManager;
import frc.robot.PathPlanner.AllianceUtils;
import frc.robot.SubsystemFactory;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.dashboard.MatchTimer;
import frc.robot.dashboard.MatchTimer.MatchPhase;

/**
 * Driver Dashboard - Shuffleboard tab with critical robot status for drivers.
 *
 * <p>Displays:
 * <ul>
 *   <li>Match timer (large, color-coded)
 *   <li>Game piece count
 *   <li>Shooter status (READY/SPINNING/ERROR)
 *   <li>Vision status (LOCKED/ODOMETRY ONLY)
 *   <li>Current action/state
 *   <li>Alliance color
 * </ul>
 *
 * <p>All subsystem access is null-safe to handle drivetrain-only testing mode.
 */
public class DriverDashboard {
    private static final String TAB_NAME = "Driver";

    private final ShuffleboardTab tab;

    private final SubsystemFactory m_subsystemFactory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final AutoManager m_autoManager;

    private final GenericEntry matchTimerEntry;
    private final GenericEntry gamePieceCountEntry;
    private final GenericEntry shooterStatusEntry;
    private final GenericEntry visionStatusEntry;
    private final GenericEntry currentActionEntry;
    private final GenericEntry allianceColorEntry;

    public DriverDashboard(
            SubsystemFactory subsystemFactory,
            CommandSwerveDrivetrain drivetrain,
            AutoManager autoManager) {
        this.m_subsystemFactory = subsystemFactory;
        this.m_drivetrain = drivetrain;
        this.m_autoManager = autoManager;

        tab = Shuffleboard.getTab(TAB_NAME);

        // Row 0: Match Timer (large, spans 4 columns)
        // Shuffleboard accepts initial values only; get entry from widget for updates.
        matchTimerEntry = tab.add("Match Timer", "--:--")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 0)
                .withSize(4, 2)
                .withProperties(java.util.Map.of("fontSize", "48"))
                .getEntry();

        // Row 1: Game Piece Count, Shooter Status
        gamePieceCountEntry = tab.add("Game Pieces", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();

        shooterStatusEntry = tab.add("Shooter Status", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 2)
                .withSize(2, 1)
                .getEntry();

        // Row 2: Vision Status, Alliance Color
        visionStatusEntry = tab.add("Vision Status", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();

        allianceColorEntry = tab.add("Alliance", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 3)
                .withSize(2, 1)
                .getEntry();

        // Row 3: Current Action
        currentActionEntry = tab.add("Current Action", "-")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 4)
                .withSize(4, 1)
                .getEntry();
    }

    /** Updates all dashboard values. Call from robotPeriodic(). */
    public void update() {
        updateMatchTimer();
        updateGamePieceCount();
        updateShooterStatus();
        updateVisionStatus();
        updateCurrentAction();
        updateAllianceColor();
        MatchTimer.logPhaseChanges();
    }

    private void updateMatchTimer() {
        double matchTime = MatchTimer.getMatchTime();
        MatchPhase phase = MatchTimer.getPhase();

        String timeString = matchTime < 0 ? "--:--" : MatchTimer.formatTime(matchTime);

        String phaseString = "";
        switch (phase) {
            case AUTO:
                phaseString = " (AUTO)";
                break;
            case TELEOP:
                phaseString = " (TELEOP)";
                break;
            case ENDGAME:
                phaseString = " (ENDGAME)";
                break;
            case DISABLED:
                phaseString = " (DISABLED)";
                break;
        }

        double timeRemaining = MatchTimer.getTeleopTimeRemaining();
        if (phase == MatchPhase.ENDGAME) {
            timeRemaining = MatchTimer.getEndgameTimeRemaining();
        } else if (phase == MatchPhase.AUTO) {
            timeRemaining = MatchTimer.getAutoTimeRemaining();
        }

        String colorPrefix;
        if (phase == MatchPhase.ENDGAME || (phase == MatchPhase.TELEOP && timeRemaining < 30.0)) {
            colorPrefix = "🔴 ";
        } else if (timeRemaining < 60.0 && phase != MatchPhase.AUTO) {
            colorPrefix = "🟡 ";
        } else {
            colorPrefix = "🟢 ";
        }

        matchTimerEntry.setValue(NetworkTableValue.makeString(colorPrefix + timeString + phaseString));
    }

    private void updateGamePieceCount() {
        // PositionTracker is commented out for drivetrain-only testing
        gamePieceCountEntry.setValue(NetworkTableValue.makeString("N/A (subsystems disabled)"));
    }

    private void updateShooterStatus() {
        // Shooter is commented out for drivetrain-only testing
        shooterStatusEntry.setValue(NetworkTableValue.makeString("N/A (subsystems disabled)"));
    }

    private void updateVisionStatus() {
        PhotonVision vision =
                m_subsystemFactory != null ? m_subsystemFactory.getPhotonVision() : null;

        if (vision != null) {
            boolean hasTargets = vision.hasTargets();
            int tagCount = 0;
            var latestResult = vision.getLatestResult();
            if (latestResult != null && latestResult.hasTargets()) {
                tagCount = latestResult.getTargets().size();
            }
            if (hasTargets && tagCount > 0) {
                visionStatusEntry.setValue(NetworkTableValue.makeString(String.format("🟢 LOCKED (%d tags)", tagCount)));
            } else {
                visionStatusEntry.setValue(NetworkTableValue.makeString("🟡 ODOMETRY ONLY"));
            }
        } else {
            visionStatusEntry.setValue(NetworkTableValue.makeString("🔴 NO CAMERA"));
        }
    }

    private void updateCurrentAction() {
        String action = "IDLE";

        if (DriverStation.isAutonomous()) {
            if (m_autoManager != null) {
                var selectedAuto = m_autoManager.getSelectedRoutine();
                if (selectedAuto != null) {
                    action = "AUTO: " + selectedAuto.getName();
                } else {
                    action = "AUTO: Running";
                }
            } else {
                action = "AUTO: Running";
            }
        }

        currentActionEntry.setValue(NetworkTableValue.makeString(action));
    }

    private void updateAllianceColor() {
        var alliance = AllianceUtils.getAlliance();
        String colorString = alliance == DriverStation.Alliance.Red ? "🔴 RED" : "🔵 BLUE";
        allianceColorEntry.setValue(NetworkTableValue.makeString(colorString));
    }
}
