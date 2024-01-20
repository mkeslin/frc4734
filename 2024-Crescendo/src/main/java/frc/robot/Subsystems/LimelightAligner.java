package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Cameras.Limelight;

public class LimelightAligner extends SubsystemBase {

    public LimelightAligner(Limelight limelight, PathPlanner pathPlanner) {}

    public Command alignToTag(int aprilTagId) {
        return Commands.runOnce(() -> {});
    }

    public Command alignToNote() {
        return Commands.runOnce(() -> {});
    }
}
