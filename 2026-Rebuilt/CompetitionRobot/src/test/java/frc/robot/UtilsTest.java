package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Unit tests for Utils class.
 * Tests utility methods for mechanism limits, soft stops, and conversions.
 */
class UtilsTest {

    @Test
    void testApplyMechanismLimits() {
        // Test upper limit switch triggered with positive voltage
        double result1 = Utils.applyMechanismLimits(5.0, false, true);
        assertEquals(0.0, result1, 0.001, 
                "Should return 0 when upper limit triggered and voltage is positive");

        // Test lower limit switch triggered with negative voltage
        double result2 = Utils.applyMechanismLimits(-5.0, true, false);
        assertEquals(0.0, result2, 0.001, 
                "Should return 0 when lower limit triggered and voltage is negative");

        // Test no limits triggered - should return voltage as-is
        double result3 = Utils.applyMechanismLimits(5.0, false, false);
        assertEquals(5.0, result3, 0.001, 
                "Should return voltage unchanged when no limits triggered");

        // Test upper limit triggered but negative voltage (should allow)
        double result4 = Utils.applyMechanismLimits(-5.0, false, true);
        assertEquals(-5.0, result4, 0.001, 
                "Should allow negative voltage when upper limit triggered");

        // Test lower limit triggered but positive voltage (should allow)
        double result5 = Utils.applyMechanismLimits(5.0, true, false);
        assertEquals(5.0, result5, 0.001, 
                "Should allow positive voltage when lower limit triggered");
    }

    @Test
    void testApplySoftStops() {
        double minPosition = 0.0;
        double maxPosition = 10.0;

        // Test at max position with positive voltage
        double result1 = Utils.applySoftStops(5.0, 10.0, minPosition, maxPosition);
        assertEquals(0.0, result1, 0.001, 
                "Should return 0 when at max position and voltage is positive");

        // Test at min position with negative voltage
        double result2 = Utils.applySoftStops(-5.0, 0.0, minPosition, maxPosition);
        assertEquals(0.0, result2, 0.001, 
                "Should return 0 when at min position and voltage is negative");

        // Test within range - should return voltage as-is
        double result3 = Utils.applySoftStops(5.0, 5.0, minPosition, maxPosition);
        assertEquals(5.0, result3, 0.001, 
                "Should return voltage unchanged when within range");

        // Test above max with negative voltage (should allow)
        double result4 = Utils.applySoftStops(-5.0, 10.0, minPosition, maxPosition);
        assertEquals(-5.0, result4, 0.001, 
                "Should allow negative voltage when at max position");

        // Test below min with positive voltage (should allow)
        double result5 = Utils.applySoftStops(5.0, 0.0, minPosition, maxPosition);
        assertEquals(5.0, result5, 0.001, 
                "Should allow positive voltage when at min position");
    }

    @Test
    void testToTwist2d() {
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 2.0, 3.0);
        Twist2d twist = Utils.toTwist2d(speeds);

        assertEquals(1.0, twist.dx, 0.001, "dx should match vx");
        assertEquals(2.0, twist.dy, 0.001, "dy should match vy");
        assertEquals(3.0, twist.dtheta, 0.001, "dtheta should match omega");
    }

    @Test
    void testInterpolateDouble() {
        double start = 0.0;
        double end = 10.0;

        // Test at start (t = 0)
        double result1 = Utils.interpolate(start, end, 0.0);
        assertEquals(0.0, result1, 0.001, "Should return start value when t = 0");

        // Test at end (t = 1)
        double result2 = Utils.interpolate(start, end, 1.0);
        assertEquals(10.0, result2, 0.001, "Should return end value when t = 1");

        // Test at midpoint (t = 0.5)
        double result3 = Utils.interpolate(start, end, 0.5);
        assertEquals(5.0, result3, 0.001, "Should return midpoint value when t = 0.5");

        // Test at quarter (t = 0.25)
        double result4 = Utils.interpolate(start, end, 0.25);
        assertEquals(2.5, result4, 0.001, "Should return quarter value when t = 0.25");
    }

    @Test
    void testInterpolateInt() {
        int start = 0;
        int end = 10;

        // Test at start (t = 0)
        int result1 = Utils.interpolate(start, end, 0.0);
        assertEquals(0, result1, "Should return start value when t = 0");

        // Test at end (t = 1)
        int result2 = Utils.interpolate(start, end, 1.0);
        assertEquals(10, result2, "Should return end value when t = 1");

        // Test at midpoint (t = 0.5)
        int result3 = Utils.interpolate(start, end, 0.5);
        assertEquals(5, result3, "Should return midpoint value when t = 0.5");
    }
}
