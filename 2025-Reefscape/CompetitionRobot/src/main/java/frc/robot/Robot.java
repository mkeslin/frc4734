package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auto.AutoManager;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public Robot() {
        // initLogging();

        AutoManager.getInstance().init();

        m_robotContainer = new RobotContainer();

        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();

        // int[] validIDs = {1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21, 22};
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-one", validIDs);

        // LiveWindow is essentially deprecated, and HoundLog is a much better
        // replacement. LiveWindow is still active during test mode by default, but it
        // consumes an inordinate amount of bandwidth, so we disable it.
        LiveWindow.disableAllTelemetry();
    }

    // private void initLogging() {
    // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    // if (isReal()) {
    // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution
    // logging
    // } else {
    // setUseTiming(false); // Run as fast as possible
    // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
    // AdvantageScope (or prompt the
    // // user)
    // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
    // "_sim"))); // Save outputs to a
    // // new log
    // }

    // Logger.start(); // Start logging! No more data receivers, replay sources, or
    // metadata values may
    // // be added.
    // }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        m_robotContainer.localizeRobotPose();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // // m_robotContainer.initializeAuto();
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // if (m_autonomousCommand != null) {
        // m_autonomousCommand.schedule();
        // }

        AutoManager.getInstance().runSelectedRoutine();

        // m_robotContainer.resetZeros();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        AutoManager.getInstance().endRoutine();

        // m_robotContainer.resetZeros();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // m_robotContainer.resetZeros();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        // m_robotContainer.resetZeros();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        // m_robotContainer.initializeTest();

        // m_robotContainer.resetZeros();

        CommandScheduler.getInstance().cancelAll();
        LiveWindow.setEnabled(false);
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {

        // m_robotContainer.resetZeros();
    }

    @Override
    public void simulationPeriodic() {
    }
}
