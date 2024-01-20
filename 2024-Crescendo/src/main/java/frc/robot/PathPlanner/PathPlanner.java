package frc.robot.PathPlanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.SwerveDrivetrain.DrivetrainConstants;
import java.util.List;

public class PathPlanner extends SubsystemBase {

    // private String pathFile = Filesystem.getDeployDirectory().getPath() + "/pathplanner/paths/Auto-1.path";
    // private String pathFile = "Auto-1";

    private CommandSwerveDrivetrain m_drivetrain;

    public PathPlanner(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    public void configure() {
        // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

        // Add a button to run pathfinding commands to SmartDashboard
        SmartDashboard.putData("Go to Start A", moveToStartA());
        SmartDashboard.putData("Go to Amp", moveToAmp());
        SmartDashboard.putData("Go to Speaker", moveToSpeaker());
        SmartDashboard.putData("Go to Source", moveToSource());
        SmartDashboard.putData("Go to Stage 1", moveToStage1());
        SmartDashboard.putData("Go to Stage 2", moveToStage2());
        SmartDashboard.putData("Go to Stage 3", moveToStage3());
        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m in the +X field direction
        // SmartDashboard.putData("On-the-fly path", moveRelative(2.0, 0.0));
    }

    public Command moveToPose(Pose2d pose) {
        var constraints = new PathConstraints(
            DrivetrainConstants.MaxSpeed,
            DrivetrainConstants.MaxAcceleration,
            Units.degreesToRadians(360),
            Units.degreesToRadians(540)
        );
        return AutoBuilder.pathfindToPose(pose, constraints, 0, 0);
    }

    public Command moveRelative(double x, double y) {
        return Commands.runOnce(() -> {
            Pose2d currentPose = m_drivetrain.getPose();

            // The rotation component in these poses represents the direction of travel
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(x, y)), new Rotation2d());

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                    DrivetrainConstants.MaxSpeed,
                    DrivetrainConstants.MaxAcceleration,
                    Units.degreesToRadians(360),
                    Units.degreesToRadians(540)
                ),
                new GoalEndState(0.0, currentPose.getRotation())
            );

            // Prevent this path from being flipped on the red alliance, since the given positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        });
    }

    public Command moveForward(double distance) {
        Pose2d currentPose = m_drivetrain.getPose();

        var y = currentPose.getRotation().getSin() * distance;
        var x = currentPose.getRotation().getCos() * distance;

        return moveRelative(x, y);
    }

    // Hard-coded
    public Command moveToStartA() {
        return moveToPose(Landmarks.StartA);
    }

    public Command moveToAmp() {
        return moveToPose(Landmarks.Amp);
    }

    public Command moveToSpeaker() {
        return moveToPose(Landmarks.Speaker);
    }

    public Command moveToSource() {
        return moveToPose(Landmarks.Source);
    }

    public Command moveToStage1() {
        return moveToPose(Landmarks.Stage1);
    }

    public Command moveToStage2() {
        return moveToPose(Landmarks.Stage2);
    }

    public Command moveToStage3() {
        return moveToPose(Landmarks.Stage3);
    }
}
