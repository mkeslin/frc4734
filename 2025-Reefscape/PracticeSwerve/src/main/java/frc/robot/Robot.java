package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public Robot() {
        // initLogging();

        m_robotContainer = new RobotContainer();

        // m_robotContainer.m_drivetrain.getDaqThread().setThreadPriority(99);

        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        /*if(m_robotContainer.hasCameras()) {
            m_robotContainer.stopCamera();
        }*/
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.initializeAuto();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        //m_robotContainer.startCamera();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        //m_robotContainer.stopCamera();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        m_robotContainer.initializeTest();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
