package frc.robot.PathPlanner;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        // SmartDashboard.putData("Go to Start A", moveToOurStartA());
        // SmartDashboard.putData("Go to Amp", moveToOurAmp());
        // SmartDashboard.putData("Go to Speaker", moveToOurSpeaker());
        // SmartDashboard.putData("Go to Source", moveToOurSource());
        // SmartDashboard.putData("Go to Stage 1", moveToOurStage1());
        // SmartDashboard.putData("Go to Stage 2", moveToOurStage2());
        // SmartDashboard.putData("Go to Stage 3", moveToOurStage3());

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m in the +X field direction
        // SmartDashboard.putData("On-the-fly path", moveRelative(2.0, 0.0));
    }

    public void resetPose(Pose2d pose) {
        m_drivetrain.resetPose(pose);
    }

    public Command moveToPose(Pose2d pose) {
        var constraints = new PathConstraints(
            DrivetrainConstants.MaxSpeed,
            DrivetrainConstants.MaxAcceleration,
            Units.degreesToRadians(100),
            Units.degreesToRadians(90)
            // Units.degreesToRadians(360),
            // Units.degreesToRadians(540)
        );
        return AutoBuilder.pathfindToPose(pose, constraints, 0, 0);
    }

    public void moveRelative(double x, double y, double rot) {
        // return Commands.runOnce(() -> {
        Pose2d currentPose = m_drivetrain.getPose();

        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(x, y)), new Rotation2d().plus(new Rotation2d(rot)));

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

        //  SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // field-centric driving in open loop
        //     .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        m_drivetrain.driveRobotRelative(new ChassisSpeeds(x, y, rot));
        // m_drivetrain.applyRequest(() -> {
        //     driveRequest.withVelocityX(x) // Drive forward with negative Y (forward)
        //         .withVelocityY(y) // Drive left with negative X (left)
        //         .withRotationalRate(rot); // Drive counterclockwise with negative X (left)

        //     return driveRequest;
        // });
    }

    public void moveForwardRobot(double distance) {
        //Pose2d currentPose = m_drivetrain.getPose();

        //var y = currentPose.getRotation().getSin() * distance;
        //var x = currentPose.getRotation().getCos() * distance;

        moveRelative(distance, 0, 0);
    }

    public void rotateRobot(double degrees) {
        double radians = degrees * Math.PI / 180;
        moveRelative(0, 0, radians);
    }

    // Hard-coded
    public Command moveToOurStartA() {
        return moveToPose(Landmarks.OurStartA());
    }

    public Command moveToOurAmp() {
        return moveToPose(Landmarks.OurAmp());
    }

    public Command moveToOurSpeaker() {
        return moveToPose(Landmarks.OurSpeaker());
    }

    public Command moveToOurSource() {
        return moveToPose(Landmarks.OurSource());
    }
    public Command moveToOurStage1() {
        return moveToPose(Landmarks.OurStage1());
    }

    public Command moveToOurStage2() {
        return moveToPose(Landmarks.OurStage2());
    }

    public Command moveToOurStage3() {
        return moveToPose(Landmarks.OurStage3());
    }

    public Command moveToOurNote1() {
        return moveToPose(Landmarks.OurNote1());
    }

    public Command moveToOurNote2() {
        return moveToPose(Landmarks.OurNote2());
    }

    public Command moveToOurNote3() {
        return moveToPose(Landmarks.OurNote3());
    }

    public Command moveToOurNote4() {
        return moveToPose(Landmarks.OurNote4());
    }

    public Command moveToOurNote5() {
        return moveToPose(Landmarks.OurNote5());
    }

    public Command moveToOurNote6() {
        return moveToPose(Landmarks.OurNote6());
    }

    public Command moveToOurNote7() {
        return moveToPose(Landmarks.OurNote7());
    }

    public Command moveToOurNote8() {
        return moveToPose(Landmarks.OurNote8());
    }

    public Command moveToBlueTest1() {
        return moveToPose(new Pose2d(2.25, 5.5, Rotation2d.fromDegrees(0)));
    }

    public Command moveToBlueTest2() {
        return moveToPose(new Pose2d(2.25, 6.5, Rotation2d.fromDegrees(0)));
    }

    public Command moveToBlueTest3() {
        return moveToPose(new Pose2d(1.25, 6.5, Rotation2d.fromDegrees(0)));
    }

    public Command moveToBlueTest4() {
        return moveToPose(new Pose2d(1.25, 5.5, Rotation2d.fromDegrees(0)));
    }

    public Command moveToRedTest1() {
        return moveToPose(new Pose2d(14.2, 5.5, Rotation2d.fromDegrees(180)));
    }

    public Command moveToRedTest2() {
        return moveToPose(new Pose2d(14.2, 4.5, Rotation2d.fromDegrees(180)));
    }

    public Command moveToRedTest3() {
        return moveToPose(new Pose2d(15.2, 4.5, Rotation2d.fromDegrees(180)));
    }

    public Command moveToRedTest4() {
        return moveToPose(new Pose2d(15.2, 5.5, Rotation2d.fromDegrees(180)));
    }
}
