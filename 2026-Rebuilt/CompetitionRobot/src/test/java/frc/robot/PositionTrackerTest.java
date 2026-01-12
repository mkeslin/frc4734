package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for PositionTracker class.
 * Tests position tracking, sensor reading, and null safety.
 */
class PositionTrackerTest {

    private PositionTracker positionTracker;
    private Supplier<Double> elevatorSupplier;
    private Supplier<Double> armSupplier;
    private Supplier<Double> sideToSideSupplier;
    private Supplier<Double> climberSupplier;
    private Supplier<Boolean> coralTraySupplier;
    private Supplier<Boolean> coralArmSupplier;
    private Supplier<Double> algaeIntakeSupplier;

    @BeforeEach
    void setUp() {
        // Create test suppliers
        elevatorSupplier = () -> 1.5;
        armSupplier = () -> 2.0;
        sideToSideSupplier = () -> 0.5;
        climberSupplier = () -> 3.0;
        coralTraySupplier = () -> false; // Sensor reads false when coral present (inverted)
        coralArmSupplier = () -> false;  // Sensor reads false when coral present (inverted)
        algaeIntakeSupplier = () -> 0.3;

        positionTracker = new PositionTracker(
                elevatorSupplier,
                armSupplier,
                sideToSideSupplier,
                climberSupplier,
                coralTraySupplier,
                coralArmSupplier,
                algaeIntakeSupplier
        );
    }

    @Test
    void testConstructorWithNullSuppliers() {
        assertThrows(NullPointerException.class, () -> {
            new PositionTracker(null, armSupplier, sideToSideSupplier, climberSupplier,
                    coralTraySupplier, coralArmSupplier, algaeIntakeSupplier);
        }, "Constructor should throw NullPointerException for null suppliers");
    }

    @Test
    void testGetElevatorPosition() {
        double position = positionTracker.getElevatorPosition();
        assertEquals(1.5, position, 0.001, "Elevator position should match supplier value");
    }

    @Test
    void testGetArmAngle() {
        double angle = positionTracker.getArmAngle();
        assertEquals(2.0, angle, 0.001, "Arm angle should match supplier value");
    }

    @Test
    void testGetSideToSidePosition() {
        double position = positionTracker.getSideToSidePosition();
        assertEquals(0.5, position, 0.001, "Side-to-side position should match supplier value");
    }

    @Test
    void testGetClimberPosition() {
        double position = positionTracker.getClimberPosition();
        assertEquals(3.0, position, 0.001, "Climber position should match supplier value");
    }

    @Test
    void testGetCoralInTray() {
        // Sensor reads false when coral is present (inverted logic)
        // So getCoralInTray() should return true when supplier returns false
        Boolean coralInTray = positionTracker.getCoralInTray();
        assertTrue(coralInTray, "getCoralInTray() should return true when sensor reads false (inverted logic)");
    }

    @Test
    void testGetCoralInArm() {
        // Sensor reads false when coral is present (inverted logic)
        // So getCoralInArm() should return true when supplier returns false
        Boolean coralInArm = positionTracker.getCoralInArm();
        assertTrue(coralInArm, "getCoralInArm() should return true when sensor reads false (inverted logic)");
    }

    @Test
    void testGetCoralInTrayWhenNoCoral() {
        // Create a tracker where sensor reads true (no coral)
        Supplier<Boolean> noCoralSupplier = () -> true;
        PositionTracker tracker = new PositionTracker(
                elevatorSupplier, armSupplier, sideToSideSupplier, climberSupplier,
                noCoralSupplier, coralArmSupplier, algaeIntakeSupplier
        );
        
        Boolean coralInTray = tracker.getCoralInTray();
        assertFalse(coralInTray, "getCoralInTray() should return false when sensor reads true (no coral)");
    }

    @Test
    void testGetAlgaeIntakeSpeed() {
        double speed = positionTracker.getAlgaeIntakeSpeed();
        assertEquals(0.3, speed, 0.001, "Algae intake speed should match supplier value");
    }

    @Test
    void testDynamicSupplierUpdates() {
        // Test that suppliers can be dynamic
        final double[] elevatorValue = {1.0};
        Supplier<Double> dynamicSupplier = () -> elevatorValue[0];
        
        PositionTracker tracker = new PositionTracker(
                dynamicSupplier, armSupplier, sideToSideSupplier, climberSupplier,
                coralTraySupplier, coralArmSupplier, algaeIntakeSupplier
        );
        
        assertEquals(1.0, tracker.getElevatorPosition(), 0.001);
        
        elevatorValue[0] = 2.0;
        assertEquals(2.0, tracker.getElevatorPosition(), 0.001, 
                "Position should update when supplier value changes");
    }
}
