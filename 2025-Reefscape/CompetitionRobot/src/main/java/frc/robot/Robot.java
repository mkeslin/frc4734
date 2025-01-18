package frc.robot;

// import org.littletonrobotics.junction.LogFileUtil;
// import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public Robot() {
        // initLogging();

        m_robotContainer = new RobotContainer();

        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
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

        // m_robotContainer.localizeRobotPose();
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
        // m_robotContainer.initializeAuto();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
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

        // m_robotContainer.initializeTest();
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
