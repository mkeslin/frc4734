package frc.robot.Auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Commands.IntakeDeployCommand;
// import frc.robot.Commands.ShootNoteCommand;
// import frc.robot.Commands.ShooterSetAngleCommand;
// import frc.robot.Subsystems.Cameras.Limelight;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
// import frc.robot.Subsystems.Climber;
// import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.Shooter;
// import java.util.ArrayList;
import java.util.List;

/*
 * Command that executes during autonomous mode
 */
public class AutoCommandTest {
    public static AutoRoutine testPath(CommandSwerveDrivetrain drivetrain) {
        PathPlannerPath path = null; //PathPlannerPath.fromPathFile("testPath");
        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("testPath", command, List.of(path), path.getStartingDifferentialPose());
    }
}
