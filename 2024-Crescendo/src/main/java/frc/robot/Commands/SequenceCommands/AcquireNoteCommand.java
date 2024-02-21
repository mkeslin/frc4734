// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.Commands.IntakeNoteCommand;
import frc.robot.Commands.MoveToNoteCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;

/*
 * Use limelight to find note
 * Move robot to correct position
 * Intake note
 */

public class AcquireNoteCommand extends SequentialCommandGroup {

    private final Limelight m_intakeLimelight;
    private final PathPlanner m_pathPlanner;
    private final Intake m_intake;

    public AcquireNoteCommand(
        Limelight intakeLimelight,
        PathPlanner pathPlanner, 
        Intake intake
    ) {
        m_intakeLimelight = intakeLimelight;
        m_pathPlanner = pathPlanner;
        m_intake = intake;

        addCommands(
            new CenterToTargetCommand(m_intakeLimelight, m_pathPlanner, 0),
            new MoveToNoteCommand(m_intakeLimelight, m_pathPlanner),
            new IntakeNoteCommand(m_intake)
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
