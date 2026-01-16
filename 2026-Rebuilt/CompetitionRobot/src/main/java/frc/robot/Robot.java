package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Logging.RobotLogger;
import frc.robot.Monitoring.PerformanceMonitor;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class Robot extends LoggedRobot { //TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private PerformanceMonitor m_performanceMonitor;

    public Robot() {
        // Initialize AdvantageKit logger
        Logger.recordMetadata("ProjectName", "FRC4734-2026-Rebuilt");
        Logger.recordMetadata("RobotType", "CompetitionRobot");
        
        // Start logging (AdvantageKit will automatically set up receivers)
        Logger.start();

        // Initialize performance monitor
        m_performanceMonitor = PerformanceMonitor.getInstance();

        m_robotContainer = new RobotContainer();

        // Warmup PathPlanner commands to reduce first-run latency
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        // PhotonVision handles AprilTag filtering through its web interface
        // No need for programmatic ID filtering like Limelight

        // LiveWindow is essentially deprecated, and AdvantageKit is a much better
        // replacement. LiveWindow is still active during test mode by default, but it
        // consumes an inordinate amount of bandwidth, so we disable it.
        LiveWindow.disableAllTelemetry();
        
        RobotLogger.log("Robot initialized");
    }

    @Override
    public void robotPeriodic() {
        // Start loop timing
        m_performanceMonitor.startLoop();
        
        // Measure command scheduler execution time
        double schedulerStart = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        double schedulerTime = Timer.getFPGATimestamp() - schedulerStart;
        m_performanceMonitor.recordSchedulerTime(schedulerTime);
        
        // Vision processing (timing measured inside localizeRobotPose)
        m_robotContainer.localizeRobotPose();
        
        // End loop timing and log metrics periodically
        m_performanceMonitor.endLoop();
    }

    @Override
    public void disabledInit() {
        // Cleanup resources when robot is disabled
        m_robotContainer.cleanup();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.getAutoManager().runSelectedRoutine();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_robotContainer.getAutoManager().endRoutine();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        LiveWindow.setEnabled(false);
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
