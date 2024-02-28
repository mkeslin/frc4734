// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Commands;
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

    public AcquireNoteCommand(Limelight intakeLimelight, PathPlanner pathPlanner, Intake intake) {
        addCommands(
            Commands.parallel(
                Commands.print("-> Center to note..."),
                new CenterToTargetCommand(intakeLimelight, pathPlanner, intake, 0),
                // Commands.print("-> Move to note..."),
                // new MoveToNoteCommand(intakeLimelight, pathPlanner, intake),
                Commands.print("-> Intake note..."),
                new IntakeNoteCommand(intake)
            )
        );
    }
}
