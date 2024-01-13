// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;

/*
 * Use limelight to find note
 * Move robot to correct position
 * Intake note
 */

public class AcquireNoteCommand extends WaitCommand {

    private final Limelight m_limelight;
    private final PathPlanner m_pathPlanner;
    private final Intake m_intake;

    public AcquireNoteCommand(Limelight limelight, PathPlanner pathPlanner, Intake intake) {
        super(2);

        m_limelight = limelight;
        m_pathPlanner = pathPlanner;
        m_intake = intake;
        
        // addRequirements(m_limelight, m_pathPlanner, m_intake);
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
