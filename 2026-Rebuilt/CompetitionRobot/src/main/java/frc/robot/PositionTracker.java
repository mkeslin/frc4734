package frc.robot;

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
    private final Supplier<Double> algaeIntakeSpeedSupplier;

    /**
     * Creates a new PositionTracker with all required suppliers.
     * 
     * @param elevatorPositionSupplier Supplier for elevator position
     * @param armAngleSupplier Supplier for arm angle
     * @param sideToSidePositionSupplier Supplier for side-to-side position
     * @param climberPositionSupplier Supplier for climber position
     * @param coralInTraySupplier Supplier for coral in tray sensor (inverted logic)
     * @param coralInArmSupplier Supplier for coral in arm sensor (inverted logic)
     * @param algaeIntakeSpeedSupplier Supplier for algae intake speed
     */
    public PositionTracker(
            Supplier<Double> elevatorPositionSupplier,
            Supplier<Double> armAngleSupplier,
            Supplier<Double> sideToSidePositionSupplier,
            Supplier<Double> climberPositionSupplier,
            Supplier<Boolean> coralInTraySupplier,
            Supplier<Boolean> coralInArmSupplier,
            Supplier<Double> algaeIntakeSpeedSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
        this.armAngleSupplier = armAngleSupplier;
        this.sideToSidePositionSupplier = sideToSidePositionSupplier;
        this.climberPositionSupplier = climberPositionSupplier;
        this.coralInTraySupplier = coralInTraySupplier;
        this.coralInArmSupplier = coralInArmSupplier;
        this.algaeIntakeSpeedSupplier = algaeIntakeSpeedSupplier;
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

    public double getAlgaeIntakeSpeed() {
        return algaeIntakeSpeedSupplier.get();
    }
}
