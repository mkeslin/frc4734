package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Logging.RobotLogger;
import frc.robot.Monitoring.PerformanceMonitor;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private PerformanceMonitor m_performanceMonitor;

    public Robot() {
        // Don't initialize anything here - wait for robotInit()
        // LoggedRobot base class handles basic setup
    }

    @Override
    public void robotInit() {
        // Initialize AdvantageKit logger
        Logger.recordMetadata("ProjectName", "FRC4734-2026-Rebuilt");
        Logger.recordMetadata("RobotType", "CompetitionRobot");
        
        // Set up data receivers before starting logger
        // NetworkTables publisher works without USB drive
        Logger.addDataReceiver(new NT4Publisher());
        
        // Optional: USB drive logging (requires USB drive mounted at /media/sda1/)
        // To enable USB logging, uncomment the following lines:
        // import org.littletonrobotics.junction.wpilog.WPILOGWriter;
        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        
        // Start logging (must be called after adding data receivers)
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

        // Update driver dashboard
        m_robotContainer.updateDriverDashboard();

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
        // Update test harness if available (for test execution)
        if (m_robotContainer != null && m_robotContainer.getAutoConfigurator() != null) {
            var testHarness = m_robotContainer.getAutoConfigurator().getTestHarness();
            if (testHarness != null) {
                testHarness.periodic();
            }
        }
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
