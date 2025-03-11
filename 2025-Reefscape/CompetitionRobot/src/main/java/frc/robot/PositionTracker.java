package frc.robot;

import java.util.function.Supplier;

public class PositionTracker {
    private Supplier<Double> elevatorPositionSupplier;
    private Supplier<Double> armAngleSupplier;
    private Supplier<Double> sideToSidePositionSupplier;
    private Supplier<Double> climberPositionSupplier;
    private Supplier<Boolean> coralInTraySupplier;
    private Supplier<Boolean> coralInArmSupplier;
    private Supplier<Double> algaeIntakeSpeedSupplier;

    public void setElevatorPositionSupplier(Supplier<Double> elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setArmAngleSupplier(Supplier<Double> armAngleSupplier) {
        this.armAngleSupplier = armAngleSupplier;
    }

    public void setSideToSidePositionSupplier(Supplier<Double> sideToSidePositionSupplier) {
        this.sideToSidePositionSupplier = sideToSidePositionSupplier;
    }

    public void setClimberPositionSupplier(Supplier<Double> climberPositionSupplier) {
        this.climberPositionSupplier = climberPositionSupplier;
    }

    public void setCoralInTraySupplier(Supplier<Boolean> coralInTray) {
        this.coralInTraySupplier = coralInTray;
    }

    public void setCoralInArmSupplier(Supplier<Boolean> coralInArm) {
        this.coralInArmSupplier = coralInArm;
    }

    public void setAlgaeIntakeSpeedSupplier(Supplier<Double> algaeIntakeSpeed) {
        this.algaeIntakeSpeedSupplier = algaeIntakeSpeed;
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

    // logic is backwards
    public Boolean getCoralInTray() {
        return !coralInTraySupplier.get();
    }

    // logic is backwards
    public Boolean getCoralInArm() {
        return !coralInArmSupplier.get();
    }

    public double getAlgaeIntakeSpeed() {
        return algaeIntakeSpeedSupplier.get();
    }
}