// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Controls.DriveController;
import frc.robot.Controls.MechanismController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modes.*;
import frc.robot.Subsystems.*;
// import frc.robot.Subsystems.AutomaticSubsystems.*;
// import frc.robot.Subsystems.Cameras.LifeCam;
import frc.robot.Subsystems.Cameras.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot documentation. 
 * 
 * If you change the name of this class or the package after creating this project, 
 * you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  // input
  private DriveController driveController;
  private MechanismController mechanismController;
  private Gyro gyro;
  // private LifeCam lifeCam;
  private Limelight limelight;
  // private UsbCamera camera1;

  // output
  private SwerveDriveRobot swerve;
  private Intake intake;
  private RotateArm rotateArm;
  private Elevator horizontalElevator;
  private Elevator verticalElevator;

  // modes
  private DisabledMode disabledMode;
  private TeleopMode teleopMode;
  private AutonomousMode autonomousMode;
  private TestMode testMode;
  private SimulationMode simulationMode;

  /*********************************************/
  // GLOBAL EVENTS
  /*********************************************/

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Controllers
    driveController = new DriveController(XC1ID);
    mechanismController = new MechanismController(XC2ID);

    // outputs
    swerve = new SwerveDriveRobot(driveController);
    intake = new Intake(mechanismController);
    rotateArm = new RotateArm(mechanismController);
    horizontalElevator = new Elevator(
        mechanismController,
        "horizontal",
        HORELEVATOR1ID,
        HORELEVATOR2ID,
        -200,
        -14000,
        -24000);
    verticalElevator = new Elevator(
        mechanismController,
        "vertical",
        VERTELEVATOR1ID,
        VERTELEVATOR2ID,
        3000,
        45000,
        45000);

    // inputs
    gyro = new Gyro();
    // lifeCam = new LifeCam();
    limelight = new Limelight();

    // mk - throwing exception
    // camera1 = CameraServer.startAutomaticCapture(0);
    // camera1.setFPS(15);
    // camera1.setResolution(640, 320);

    swerve.setMode(NeutralMode.Coast);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    swerve.putNums();
    Limelight.putNums();
    gyro.putNums();
  }

  /*********************************************/
  // MODE EVENTS - DISABLED
  /*********************************************/

  /** This function is called when mode starts. */
  @Override
  public void disabledInit() {
    disabledMode = new DisabledMode(swerve);
    disabledMode.init();
  }

  /** This function is called periodically. */
  @Override
  public void disabledPeriodic() {
    disabledMode.periodic();
  }

  /*********************************************/
  // MODE EVENTS - AUTONOMOUS
  /*********************************************/

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  /** This function is called when mode starts. */
  @Override
  public void autonomousInit() {
    autonomousMode = new AutonomousMode(driveController, swerve, intake, rotateArm, horizontalElevator,
        verticalElevator, limelight, gyro);
    autonomousMode.init();
  }

  /** This function is called periodically. */
  @Override
  public void autonomousPeriodic() {
    autonomousMode.periodic();
  }

  /*********************************************/
  // MODE EVENTS - TELEOP
  /*********************************************/

  /** This function is called when mode starts. */
  @Override
  public void teleopInit() {
    teleopMode = new TeleopMode(swerve, intake, rotateArm, horizontalElevator, verticalElevator);
    teleopMode.init();
  }

  /** This function is called periodically. */
  @Override
  public void teleopPeriodic() {
    teleopMode.periodic();
  }

  /*********************************************/
  // MODE EVENTS - TEST
  /*********************************************/

  /** This function is called when mode starts. */
  @Override
  public void testInit() {
    testMode = new TestMode(intake);
    testMode.init();
  }

  /** This function is called periodically. */
  @Override
  public void testPeriodic() {
    testMode.periodic();
  }

  /*********************************************/
  // MODE EVENTS - SIMULATION
  /*********************************************/

  /** This function is called when mode starts. */
  @Override
  public void simulationInit() {
    simulationMode = new SimulationMode();
    simulationMode.init();
  }

  /** This function is called periodically. */
  @Override
  public void simulationPeriodic() {
    simulationMode.periodic();
  }
}
