package frc.robot.SwerveDrivetrain;

import frc.robot.Subsystems.DeployableIntake;

/**
 * Whether to scale down drivetrain translation while the intake roller is running
 * ({@link DeployableIntake#isRollerRunningForDriveThrottle()}).
 */
public final class IntakeDriveThrottle {

    private IntakeDriveThrottle() {}

    /**
     * @param intake deployable intake; if null, never throttles
     */
    public static boolean shouldThrottleDriveTranslation(DeployableIntake intake) {
        return intake != null && intake.isRollerRunningForDriveThrottle();
    }
}
