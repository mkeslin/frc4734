package frc.robot;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * Tracks the positions and states of all robot mechanisms.
 * Uses constructor injection to ensure all suppliers are provided at creation time,
 * preventing null pointer exceptions.
 */
public class PositionTracker {
    private final Supplier<Double> elevatorPositionSupplier;
    private final Supplier<Double> armAngleSupplier;
    private final Supplier<Double> sideToSidePositionSupplier;
    private final Supplier<Double> climberPositionSupplier;
    private final Supplier<Boolean> coralInTraySupplier;
    private final Supplier<Boolean> coralInArmSupplier;
    private final Supplier<Double> intakeDeployPositionSupplier;
    private final Supplier<Double> intakeSpeedSupplier;
    private final Supplier<Double> floorSpeedSupplier;
    private final Supplier<Double> feederSpeedSupplier;
    private final Supplier<Double> shooterSpeedSupplier;

    /**
     * Creates a new PositionTracker with all required suppliers.
     * 
     * @param elevatorPositionSupplier Supplier for elevator position
     * @param armAngleSupplier Supplier for arm angle
     * @param sideToSidePositionSupplier Supplier for side-to-side position
     * @param climberPositionSupplier Supplier for climber position
     * @param coralInTraySupplier Supplier for coral in tray sensor (inverted logic)
     * @param coralInArmSupplier Supplier for coral in arm sensor (inverted logic)
     * @param intakeDeployPositionSupplier Supplier for intake deploy position
     * @param intakeSpeedSupplier Supplier for intake speed
     * @param floorSpeedSupplier Supplier for floor conveyor speed
     * @param feederSpeedSupplier Supplier for feeder speed
     * @param shooterSpeedSupplier Supplier for shooter speed
     */
    public PositionTracker(
            Supplier<Double> elevatorPositionSupplier,
            Supplier<Double> armAngleSupplier,
            Supplier<Double> sideToSidePositionSupplier,
            Supplier<Double> climberPositionSupplier,
            Supplier<Boolean> coralInTraySupplier,
            Supplier<Boolean> coralInArmSupplier,
            Supplier<Double> intakeDeployPositionSupplier,
            Supplier<Double> intakeSpeedSupplier,
            Supplier<Double> floorSpeedSupplier,
            Supplier<Double> feederSpeedSupplier,
            Supplier<Double> shooterSpeedSupplier) {
        this.elevatorPositionSupplier = Objects.requireNonNull(elevatorPositionSupplier, "elevatorPositionSupplier cannot be null");
        this.armAngleSupplier = Objects.requireNonNull(armAngleSupplier, "armAngleSupplier cannot be null");
        this.sideToSidePositionSupplier = Objects.requireNonNull(sideToSidePositionSupplier, "sideToSidePositionSupplier cannot be null");
        this.climberPositionSupplier = Objects.requireNonNull(climberPositionSupplier, "climberPositionSupplier cannot be null");
        this.coralInTraySupplier = Objects.requireNonNull(coralInTraySupplier, "coralInTraySupplier cannot be null");
        this.coralInArmSupplier = Objects.requireNonNull(coralInArmSupplier, "coralInArmSupplier cannot be null");
        this.intakeDeployPositionSupplier = Objects.requireNonNull(intakeDeployPositionSupplier, "intakeDeployPositionSupplier cannot be null");
        this.intakeSpeedSupplier = Objects.requireNonNull(intakeSpeedSupplier, "intakeSpeedSupplier cannot be null");
        this.floorSpeedSupplier = Objects.requireNonNull(floorSpeedSupplier, "floorSpeedSupplier cannot be null");
        this.feederSpeedSupplier = Objects.requireNonNull(feederSpeedSupplier, "feederSpeedSupplier cannot be null");
        this.shooterSpeedSupplier = Objects.requireNonNull(shooterSpeedSupplier, "shooterSpeedSupplier cannot be null");
    }

    public double getElevatorPosition() {
        return elevatorPositionSupplier.get();
    }

    public double getArmAngle() {
        return armAngleSupplier.get();
    }

    public double getSideToSidePosition() {
        return sideToSidePositionSupplier.get();
    }

    public double getClimberPosition() {
        return climberPositionSupplier.get();
    }

    /**
     * Gets whether coral is in the tray.
     * Note: Logic is inverted - returns true when sensor reads false.
     * 
     * @return true if coral is in tray, false otherwise
     */
    public Boolean getCoralInTray() {
        return !coralInTraySupplier.get();
    }

    /**
     * Gets whether coral is in the arm.
     * Note: Logic is inverted - returns true when sensor reads false.
     * 
     * @return true if coral is in arm, false otherwise
     */
    public Boolean getCoralInArm() {
        return !coralInArmSupplier.get();
    }

    public double getIntakeDeployPosition() {
        return intakeDeployPositionSupplier.get();
    }

    public double getIntakeSpeed() {
        return intakeSpeedSupplier.get();
    }

    public double getFloorSpeed() {
        return floorSpeedSupplier.get();
    }

    public double getFeederSpeed() {
        return feederSpeedSupplier.get();
    }

    public double getShooterSpeed() {
        return shooterSpeedSupplier.get();
    }
}
