package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.AutoCommandA;
import frc.robot.Auto.AutoManager;
import frc.robot.Commands.CenterToReefCommand;
import frc.robot.Commands.RobotContext;

/**
 * Configures autonomous routines for the robot.
 * Centralizes autonomous setup logic.
 */
public class AutoConfigurator {
    private final RobotContext m_robotContext;
    private final CenterToReefCommand m_centerToReefCommand;

    /**
     * Creates a new AutoConfigurator with required dependencies.
     * 
     * @param robotContext The robot context containing all subsystems
     * @param centerToReefCommand The center to reef command
     */
    public AutoConfigurator(RobotContext robotContext, CenterToReefCommand centerToReefCommand) {
        m_robotContext = robotContext;
        m_centerToReefCommand = centerToReefCommand;
    }

    /**
     * Configures all autonomous routines and registers them with AutoManager.
     */
    public void configureAuto() {
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition1(m_robotContext, m_centerToReefCommand));
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition2(m_robotContext, m_centerToReefCommand));
        AutoManager.getInstance()
                .addRoutine(AutoCommandA.StartingPosition3(m_robotContext, m_centerToReefCommand));

        SmartDashboard.putData("Auto Mode (manager)", AutoManager.getInstance().chooser);
    }
}
