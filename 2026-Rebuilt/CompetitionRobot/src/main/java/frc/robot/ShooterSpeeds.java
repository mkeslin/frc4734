package frc.robot;

import static frc.robot.Constants.ShooterConstants.SHOOTER_CENTER_SPEED_MULTIPLIER;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_SPEED_MULTIPLIER;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_SPEED_MULTIPLIER;

/**
 * Per-motor shooter speeds (RPS) for left, center, and right motors.
 * Values are magnitudes; the Shooter subsystem applies sign convention
 * (left/center negative, right positive for forward spin).
 */
public record ShooterSpeeds(double leftRps, double centerRps, double rightRps) {

    /**
     * Creates ShooterSpeeds from a single base RPS using per-motor multipliers.
     * Teleop only. Auto always uses explicit per-motor values (no multipliers).
     */
    public static ShooterSpeeds fromBase(double baseRps) {
        return new ShooterSpeeds(
                baseRps * SHOOTER_LEFT_SPEED_MULTIPLIER,
                baseRps * SHOOTER_CENTER_SPEED_MULTIPLIER,
                baseRps * SHOOTER_RIGHT_SPEED_MULTIPLIER);
    }

    /**
     * Creates ShooterSpeeds with the same RPS for all three motors. No multipliers.
     * For auto use only; teleop uses fromBase() with per-motor multipliers.
     */
    public static ShooterSpeeds uniform(double rps) {
        return new ShooterSpeeds(rps, rps, rps);
    }
}
