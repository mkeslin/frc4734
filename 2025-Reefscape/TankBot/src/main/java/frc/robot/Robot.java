package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Constants;

public class Robot extends TimedRobot {

    // private Command m_autonomousCommand;

    // private RobotContainer m_robotContainer;

    private CANSparkMax m_leftMotor = new CANSparkMax(0, MotorType.kBrushed);
    private CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushed);
    // private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
    // private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    private final XboxController m_driverController = new XboxController(0);

    @Override
    public void robotInit() {
        // m_robotContainer = new RobotContainer();

        // m_robotContainer.m_drivetrain.getDaqThread().setThreadPriority(99);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotor.setInverted(true);
    }

    @Override
    public void robotPeriodic() {
        // CommandScheduler.getInstance().run();
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
        // m_robotContainer.initializeAuto();
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.schedule();
        // }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.cancel();
        // }
        // //m_robotContainer.startCamera();
    }

    @Override
    public void teleopPeriodic() {
        // Drive with tank drive.
        // That means that the Y axis of the left stick moves the left side
        // of the robot forward and backward, and the Y axis of the right stick
        // moves the right side of the robot forward and backward.
        m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());
    }

    @Override
    public void teleopExit() {
        //m_robotContainer.stopCamera();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        // m_robotContainer.initializeTest();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
