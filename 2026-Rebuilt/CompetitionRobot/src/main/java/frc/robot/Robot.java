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
        AutoManager.getInstance().init();

        m_robotContainer = new RobotContainer();

        // Warmup PathPlanner commands to reduce first-run latency
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        int[] validIDs = {1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21, 22};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        // LiveWindow is essentially deprecated, and HoundLog is a much better
        // replacement. LiveWindow is still active during test mode by default, but it
        // consumes an inordinate amount of bandwidth, so we disable it.
        LiveWindow.disableAllTelemetry();
    }

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
        AutoManager.getInstance().runSelectedRoutine();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        AutoManager.getInstance().endRoutine();
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
