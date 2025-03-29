package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandTest {
    public static AutoRoutine testPath(CommandSwerveDrivetrain drivetrain) {
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("testPath");
        } catch (Exception exception) {
            DriverStation.reportError("[AutoCommandA]: " + exception.getMessage(), false);
        }
        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("testPath", command, List.of(path), path.getStartingDifferentialPose());
    }
}
