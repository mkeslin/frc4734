// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SequenceCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CenterToTargetCommand;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.River;

/*
 * Use limelight to find note
 * Move robot to correct position
 * Intake note
 */

public class AcquireNoteCommand extends SequentialCommandGroup {

    public AcquireNoteCommand(Limelight intakeLimelight, PathPlanner pathPlanner, Intake intake, River river, double speed) {
        addCommands(
            Commands.parallel(
                Commands.print("-> Center to note and acquire..."),
                new CenterToTargetCommand(intakeLimelight, pathPlanner, intake, river, 0, speed)
                // I don't think this is needed, since the above command also moves towards note
                // Commands.print("-> Move to note..."),
                // new MoveToNoteCommand(intakeLimelight, pathPlanner, intake),

                // first command also starts intake
                // Commands.print("-> Intake note..."),
                // new IntakeNoteCommand(intake)
            )
        );
    }
}
