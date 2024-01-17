package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;

public class AprilTagAligner {

    public AprilTagAligner(Limelight limelight, PathPlanner pathPlanner) {
        
    }

    public Command AlignToTag(int aprilTagId) {
        return Commands.runOnce(() -> {});
    }
}
