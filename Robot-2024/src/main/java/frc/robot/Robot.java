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
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modes.*;
import frc.robot.Subsystems.*;
// import frc.robot.Subsystems.AutomaticSubsystems.*;
// import frc.robot.Subsystems.Cameras.LifeCam;
import frc.robot.Subsystems.Cameras.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  // input
  private XboxController driverController, mechanismController;
  private Gyro gyro;
  // private LifeCam lifeCam;
  private Limelight limelight;
  // private UsbCamera camera1;

  // output
  private SwerveDrive swerve;
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
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Controllers
    driverController = new XboxController(XC1ID);
    mechanismController = new XboxController(XC2ID);

    // outputs
    swerve = new SwerveDrive(driverController);
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
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    swerve.putNums();
    Limelight.putNums();
    gyro.putNums();
  }

  /*********************************************/
  // DISABLED EVENTS
  /*********************************************/

  /* This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    disabledMode = new DisabledMode(swerve);
    disabledMode.init();
  }

  /* This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    disabledMode.periodic();
  }

  /*********************************************/
  // AUTONOMOUS EVENTS
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
  @Override
  public void autonomousInit() {
    autonomousMode = new AutonomousMode(swerve, intake, rotateArm, horizontalElevator, verticalElevator, limelight,
        gyro);
    autonomousMode.init();
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autonomousMode.periodic();
  }

  /*********************************************/
  // TELEOP EVENTS
  /*********************************************/

  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    teleopMode = new TeleopMode(swerve, intake, rotateArm, horizontalElevator, verticalElevator);
    teleopMode.init();
  }

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    teleopMode.periodic();
  }

  /*********************************************/
  // TEST EVENTS
  /*********************************************/

  /* This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    testMode = new TestMode(intake);
    testMode.init();
  }

  /* This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    testMode.periodic();
  }

  /*********************************************/
  // SIMULATION EVENTS
  /*********************************************/

  /* This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    simulationMode = new SimulationMode();
    simulationMode.init();
  }

  /* This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    simulationMode.periodic();
  }

  /*********************************************/
  // HELPERS
  /*********************************************/

  // private void zeroMotors() {
  // swerve.zeroMotors();
  // }

  // private void setSwerve() {
  // if (driverController.getRawButton(CLB))
  // swerve.setMode(NeutralMode.Brake);
  // else if (driverController.getRawButton(CRB))
  // swerve.setMode(NeutralMode.Coast);
  // else
  // swerve.controllerDrive();
  // if (driverController.getRawAxis(CLT) > 0.5)
  // swerve.setSpeedFactor(0.5);
  // if (driverController.getRawAxis(CRT) > 0.5)
  // swerve.setSpeedFactor(1);
  // }

  // private void setIntake() {
  // if (mechanismController.getRawAxis(CLT) > 0.5) {
  // intake.wheelsIn();
  // } else if (mechanismController.getRawAxis(CRT) > 0.5) {
  // intake.wheelsOut();
  // } else {
  // intake.zero();
  // }

  // if (mechanismController.getRawButton(CLB)) {
  // intake.open();
  // } else if (mechanismController.getRawButton(CRB)) {
  // intake.close();
  // }
  // }

  // private void setRotateArm() {
  // if (mechanismController.getRawAxis(CLY) < -0.5) {
  // rotateArm.extend(Math.abs(mechanismController.getRawAxis(CLY)));
  // } else if (mechanismController.getRawAxis(CLY) > 0.5) {
  // rotateArm.retract(Math.abs(mechanismController.getRawAxis(CLY)));
  // } else {
  // rotateArm.zero();
  // }
  // }

  // private void setElevators() {
  // if (mechanismController.getRawAxis(CRY) < -0.5 ||
  // horizontalElevator.getElevatorMovingOut()) {
  // horizontalElevator.movePositive();
  // }
  // if (mechanismController.getRawAxis(CRY) > 0.5 ||
  // horizontalElevator.getElevatorMovingIn()) {
  // horizontalElevator.moveNegative();
  // }
  // if (!horizontalElevator.getElevatorMovingOut() &&
  // !horizontalElevator.getElevatorMovingIn()) {
  // horizontalElevator.zero();
  // }

  // if (mechanismController.getRawButtonPressed(CYB) ||
  // verticalElevator.getElevatorMovingOut()) {
  // verticalElevator.movePositive();
  // }
  // if (mechanismController.getRawButtonPressed(CAB) ||
  // verticalElevator.getElevatorMovingIn()) {
  // verticalElevator.moveNegative();
  // }
  // if (!verticalElevator.getElevatorMovingOut() &&
  // !verticalElevator.getElevatorMovingIn()) {
  // verticalElevator.zero();
  // }
  // }

  // private void setScoreHigh() {
  // if (driverController.getRawButton(CAB) && !auto.getScoringHigh()) {
  // auto.changeState();
  // auto.scoreHigh();
  // }
  // if (auto.getScoringHigh()) {
  // auto.scoreHigh();
  // }
  // }
}
