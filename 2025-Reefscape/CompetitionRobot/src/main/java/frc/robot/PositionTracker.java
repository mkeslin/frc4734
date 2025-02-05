package frc.robot;

import java.util.function.Supplier;

public class PositionTracker {
    private Supplier<Double> elevatorPositionSupplier;
    private Supplier<Double> armAngleSupplier;
    private Supplier<Double> sideToSidePositionSupplier;
    private Supplier<Double> climberPositionSupplier;
    private Supplier<Boolean> coralInArm;

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

    public void setCoralInArm(Supplier<Boolean> coralInArm) {
        this.coralInArm = coralInArm;
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

    public Boolean getCoralInArm() {
        return coralInArm.get();
    }
}