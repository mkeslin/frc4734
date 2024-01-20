// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

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

public class AcquireNoteCommand extends SequentialCommandGroup {

    // private final Limelight m_limelight;
    // private final PathPlanner m_pathPlanner;
    // private final Intake m_intake;

    public AcquireNoteCommand(
        Limelight limelight, 
        PathPlanner pathPlanner, 
        Intake intake, 
        LimelightAligner limelightAligner
    ) {
        // super(2);

        // m_limelight = limelight;
        // m_pathPlanner = pathPlanner;
        // m_intake = intake;

        // addRequirements(m_limelight, m_pathPlanner, m_intake);

        addCommands(
            // use limelight to position robot in front of note
            // limelightAligner.alignToTag(),
            // wait
            Commands.waitSeconds(2),
            // start intake motors
            intake.commandStartIn(),
            // drive forward to intake note
            pathPlanner.moveForward(.2),
            // wait
            Commands.waitSeconds(2),
            // stop intake motors
            intake.commandStop()
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
