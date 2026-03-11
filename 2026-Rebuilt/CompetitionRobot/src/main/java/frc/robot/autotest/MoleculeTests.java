package frc.robot.autotest;

import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Auto.commands.AutoConstants;
import frc.robot.Auto.commands.CmdAcquireHubAim;
import frc.robot.Auto.commands.CmdApplyTagSnapIfGood;
import frc.robot.Auto.commands.CmdClimbL1;
import frc.robot.Auto.commands.CmdDriveToPose;
import frc.robot.Auto.commands.CmdFollowPath;
import frc.robot.Auto.commands.CmdHoldClimbUntilEnd;
import frc.robot.Auto.commands.CmdSeedOdometryFromStartPose;
import frc.robot.Auto.commands.CmdShooterSpinUp;
import frc.robot.Auto.commands.CmdShootForTime;
import frc.robot.Auto.commands.CmdWaitShooterAtSpeed;
import frc.robot.Auto.commands.StartPoseId;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Floor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Factory class for creating pre-defined molecule tests (2-3 command sequences).
 * 
 * <p>Molecules test composition of atomic commands to verify they work together.
 * Each molecule has explicit timeouts for each step and can be run from any start pose.
 */
public final class MoleculeTests {
    private MoleculeTests() {
        // Utility class - prevent instantiation
    }

    /**
     * Builds molecule: Seed->PathToShot->TagSnap->AcquireAim
     * 
     * <p>Tests the sequence of seeding odometry, following a path to shooting position,
     * applying tag snap for pose correction, and acquiring hub aim.
     * 
     * @param startPoseId Selected start pose
     * @param startPoseSuppliers Map of start pose IDs to suppliers (evaluated at runtime for correct alliance)
     * @param pathName Path name (e.g., "L_StartToShot" for POS_1)
     * @param fallbackHeadingDeg Fallback heading for hub aim
     * @param drivetrain Drivetrain subsystem
     * @param vision Vision subsystem
     * @return Command sequence
     */
    public static Command buildSeedPathTagSnapAim(
            StartPoseId startPoseId,
            Map<StartPoseId, Supplier<Pose2d>> startPoseSuppliers,
            String pathName,
            double fallbackHeadingDeg,
            CommandSwerveDrivetrain drivetrain,
            PhotonVision vision) {
        
        Objects.requireNonNull(startPoseId, "startPoseId cannot be null");
        Objects.requireNonNull(startPoseSuppliers, "startPoseSuppliers cannot be null");
        Objects.requireNonNull(pathName, "pathName cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(vision, "vision cannot be null");
        
        return Commands.sequence(
                // 1. Seed odometry
                CmdSeedOdometryFromStartPose.create(startPoseId, startPoseSuppliers, drivetrain),
                
                // 2. Follow path to shot
                new CmdFollowPath(pathName, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                
                // 3. Tag snap
                new CmdApplyTagSnapIfGood(
                        vision,
                        drivetrain,
                        AutoConstants.DEFAULT_MAX_AMBIGUITY,
                        AutoConstants.DEFAULT_MAX_TAG_DISTANCE,
                        AutoConstants.DEFAULT_MIN_TARGETS),
                
                // 4. Acquire hub aim
                CmdAcquireHubAim.create(vision, drivetrain, fallbackHeadingDeg)
        ).withName("Mol: Seed->Path->TagSnap->Aim");
    }

    /**
     * Builds molecule: PathToShot + SpinUp (parallel) -> ShootPreload
     *
     * @param pathName Path name to shooting position
     * @param shooterSpeedsSupplier Supplier of per-motor shooter speeds
     * @param rpmTol RPM tolerance
     * @param shootDuration Duration to shoot
     * @param drivetrain Drivetrain subsystem
     * @param shooter Shooter subsystem
     * @param feeder Feeder subsystem
     * @param floor Optional floor subsystem; if non-null, runs with feeder to feed notes
     * @return Command sequence
     */
    public static Command buildPathSpinupShoot(
            String pathName,
            java.util.function.Supplier<frc.robot.ShooterSpeeds> shooterSpeedsSupplier,
            double rpmTol,
            double shootDuration,
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            Feeder feeder,
            Floor floor) {
        
        Objects.requireNonNull(pathName, "pathName cannot be null");
        Objects.requireNonNull(shooterSpeedsSupplier, "shooterSpeedsSupplier cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(shooter, "shooter cannot be null");
        Objects.requireNonNull(feeder, "feeder cannot be null");
        
        return Commands.sequence(
                // 1. Follow path to shot (parallel: spin up shooter)
                Commands.parallel(
                        new CmdFollowPath(pathName, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                        new CmdShooterSpinUp(shooter, shooterSpeedsSupplier)),
                
                // 2. Wait for shooter at speed
                CmdWaitShooterAtSpeed.create(shooter, shooterSpeedsSupplier, rpmTol),
                
                // 3. Shoot for duration
                CmdShootForTime.create(shooter, feeder, floor, shootDuration)
        ).withName("Mol: Path+SpinUp->Shoot");
    }

    /**
     * Builds molecule: PathToTowerStage->DriveToPoseAlign->Climb->Hold
     * 
     * <p>Tests the complete climb sequence: path to tower, align to pose, climb, and hold.
     * 
     * @param pathName Path name to tower stage
     * @param alignPoseSupplier Supplier of target pose for alignment (evaluated at runtime for correct alliance)
     * @param drivetrain Drivetrain subsystem
     * @param climber Climber subsystem
     * @return Command sequence
     */
    public static Command buildPathAlignClimb(
            String pathName,
            Supplier<Pose2d> alignPoseSupplier,
            CommandSwerveDrivetrain drivetrain,
            Climber climber) {
        
        Objects.requireNonNull(pathName, "pathName cannot be null");
        Objects.requireNonNull(alignPoseSupplier, "alignPoseSupplier cannot be null");
        Objects.requireNonNull(drivetrain, "drivetrain cannot be null");
        Objects.requireNonNull(climber, "climber cannot be null");
        
        return Commands.sequence(
                // 1. Follow path to tower stage
                new CmdFollowPath(pathName, AutoConstants.DEFAULT_PATH_TIMEOUT, drivetrain),
                
                // 2. Drive to alignment pose
                CmdDriveToPose.create(
                        drivetrain,
                        alignPoseSupplier,
                        AutoConstants.DEFAULT_XY_TOLERANCE,
                        AutoConstants.DEFAULT_ROTATION_TOLERANCE,
                        AutoConstants.DEFAULT_POSE_TIMEOUT),
                
                // 3. Climb L1
                CmdClimbL1.create(climber),
                
                // 4. Hold climb until end
                new CmdHoldClimbUntilEnd(climber, drivetrain)
        ).withName("Mol: Path->Align->Climb->Hold");
    }

    /**
     * Helper method to get path name based on start pose.
     * Uses convention: "{POS}_PathName" where POS is L/C/R for POS_1/POS_2/POS_3
     * 
     * @param startPoseId Start pose ID
     * @param basePathName Base path name (e.g., "StartToShot")
     * @return Full path name (e.g., "L_StartToShot")
     */
    public static String getPathNameForPose(StartPoseId startPoseId, String basePathName) {
        String prefix;
        switch (startPoseId) {
            case POS_1:
                prefix = "L";
                break;
            case POS_2:
                prefix = "C";
                break;
            case POS_3:
                prefix = "R";
                break;
            default:
                prefix = "L";
        }
        return prefix + "_" + basePathName;
    }
}
