package frc.robot;

import java.util.function.Supplier;

public class PositionTracker {
    private Supplier<Double> elevatorPositionSupplier;
    private Supplier<Double> armAngleSupplier;
    private Supplier<Double> sideToSidePositionSupplier;

    public void setElevatorPositionSupplier(Supplier<Double> elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setArmAngleSupplier(Supplier<Double> armAngleSupplier) {
        this.armAngleSupplier = armAngleSupplier;
    }

    public void setSideToSidePositionSupplier(Supplier<Double> sideToSidePositionSupplier) {
        this.sideToSidePositionSupplier = sideToSidePositionSupplier;
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
}