// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LimelightAligner;

/*
 * Use limelight to find note
 * Move robot to correct position
 * Intake note
 */

public class AutoCommand extends SequentialCommandGroup {

    // private final Limelight m_limelight;
    // private final PathPlanner m_pathPlanner;
    // private final Intake m_intake;

    public AutoCommand(
        // Limelight shooterLimelight, 
        // Limelight intakeLimelight,
        PathPlanner pathPlanner
        // Intake intake, 
        // LimelightAligner limelightAligner
    ) {
        // super(2);

        // m_limelight = limelight;
        // m_pathPlanner = pathPlanner;
        // m_intake = intake;

        // addRequirements(m_limelight, m_pathPlanner, m_intake);

        SmartDashboard.putString("auto command", "I'm here!!!");
        System.out.println("autoCommand ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

        addCommands(

            pathPlanner.moveToRing1()
            // limelightAligner

            // use limelight to position robot in front of note
            // limelightAligner.alignToTag(),
            // wait
            // Commands.print("acquireNote-111111111111111111111111111111111111111"),
            // Commands.waitSeconds(2),
            // Commands.print("acquireNote-222222222222222222222222222222222222222"),
            // start intake motors
            // intake.commandStartIn(),
            // drive forward to intake note
            //pathPlanner.moveForwardRobot(.2),
            // wait
            // Commands.waitSeconds(2),
            // Commands.print("acquireNote-333333333333333333333333333333333333333")
            // stop intake motors
            // intake.commandStop()
        );
    }
    // Called just before this Command runs the first time
    // @Override
    // public void initialize() {}

    // Make this return true when this Command no longer needs to run execute()
    // @Override
    // public boolean isFinished() {
    //     // return m_claw.isGrabbing();
    //     return true;
    // }

    // Called once after isFinished returns true
    // @Override
    // public void end(boolean interrupted) {}
}