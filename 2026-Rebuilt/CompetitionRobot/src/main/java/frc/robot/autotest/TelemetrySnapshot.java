package frc.robot.autotest;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DeployableIntake;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Utility class for capturing telemetry snapshots from robot subsystems.
 * Provides a consistent way to record robot state at specific points in time.
 */
public final class TelemetrySnapshot {
    private TelemetrySnapshot() {
        // Utility class - prevent instantiation
    }

    /**
     * Captures a snapshot of current subsystem telemetry.
     * 
     * @param drive The drivetrain subsystem (required)
     * @param vision The vision subsystem (optional)
     * @param shooter The shooter subsystem (optional)
     * @param feeder The feeder subsystem (optional)
     * @param intake The intake subsystem (optional)
     * @param climber The climber subsystem (optional)
     * @param positionTracker The position tracker (optional)
     * @param headingTargetOpt Optional target heading for error calculation
     * @return Map of telemetry values (all keys present, null-safe defaults)
     */
    public static Map<String, Double> capture(
            CommandSwerveDrivetrain drive,
            PhotonVision vision,
            Shooter shooter,
            Feeder feeder,
            DeployableIntake intake,
            Climber climber,
            PositionTracker positionTracker,
            Optional<Rotation2d> headingTargetOpt) {
        
        Map<String, Double> metrics = new HashMap<>();
        
        // Drivetrain pose
        if (drive != null) {
            Pose2d pose = drive.getPose();
            metrics.put("poseX", pose.getX());
            metrics.put("poseY", pose.getY());
            metrics.put("poseDeg", pose.getRotation().getDegrees());
            metrics.put("headingDeg", pose.getRotation().getDegrees());
        } else {
            metrics.put("poseX", 0.0);
            metrics.put("poseY", 0.0);
            metrics.put("poseDeg", 0.0);
            metrics.put("headingDeg", 0.0);
        }
        
        // Heading error (if target provided)
        if (headingTargetOpt.isPresent() && drive != null) {
            Rotation2d currentHeading = drive.getPose().getRotation();
            Rotation2d targetHeading = headingTargetOpt.get();
            double error = Math.abs(currentHeading.minus(targetHeading).getRadians());
            // Normalize to [-pi, pi]
            error = Math.min(error, 2 * Math.PI - error);
            metrics.put("headingErrorDeg", Math.toDegrees(error));
        } else {
            metrics.put("headingErrorDeg", 0.0);
        }
        
        // Vision metrics
        if (vision != null) {
            var result = vision.getLatestResult();
            if (result != null && result.hasTargets()) {
                metrics.put("tagCount", (double) result.getTargets().size());
                
                // Calculate average tag distance
                var targets = result.getTargets();
                double totalDistance = 0.0;
                for (var target : targets) {
                    totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                metrics.put("tagDistance", totalDistance / targets.size());
                
                // Ambiguity (if available from EstimatedRobotPose)
                var estimatedPose = vision.getEstimatedRobotPose();
                if (estimatedPose.isPresent()) {
                    // Note: ambiguity may not be available in all PhotonVision versions
                    // Using 0.0 as default if not available
                    metrics.put("tagAmbiguity", 0.0); // Placeholder - PhotonVision may not expose this directly
                } else {
                    metrics.put("tagAmbiguity", 1.0); // No pose = high ambiguity
                }
            } else {
                metrics.put("tagCount", 0.0);
                metrics.put("tagDistance", -1.0);
                metrics.put("tagAmbiguity", 1.0);
            }
        } else {
            metrics.put("tagCount", 0.0);
            metrics.put("tagDistance", -1.0);
            metrics.put("tagAmbiguity", 1.0);
        }
        
        // Shooter metrics
        if (shooter != null) {
            double actualRpm = shooter.getSpeed();
            metrics.put("shooterRpmActual", actualRpm);
            // Setpoint would need to be tracked separately - using actual as placeholder
            metrics.put("shooterRpmSetpoint", actualRpm);
            // At speed check (would need target RPM to compare - using 0/1 based on non-zero)
            metrics.put("shooterAtSpeed", actualRpm > 0.1 ? 1.0 : 0.0);
        } else {
            metrics.put("shooterRpmActual", 0.0);
            metrics.put("shooterRpmSetpoint", 0.0);
            metrics.put("shooterAtSpeed", 0.0);
        }
        
        // Ball count (using PositionTracker sensor)
        if (positionTracker != null) {
            Boolean ballInTray = positionTracker.getCoralInTray();
            // Convert boolean to count (0 or 1, or -1 if unavailable)
            metrics.put("ballCount", ballInTray != null && ballInTray ? 1.0 : 0.0);
        } else {
            metrics.put("ballCount", -1.0);
        }
        
        // Climb qualified (checks if climber is near CLIMB position)
        if (climber != null) {
            double currentPosition = climber.getPosition();
            double targetPosition = ClimberPosition.CLIMB.value;
            double positionError = Math.abs(currentPosition - targetPosition);
            metrics.put("climbQualified", positionError < 1.0 ? 1.0 : 0.0);
        } else {
            metrics.put("climbQualified", 0.0);
        }
        
        return metrics;
    }
}
