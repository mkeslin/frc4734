package frc.robot.Constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for CommandConstants class.
 * Verifies that constants are properly defined and have expected values.
 */
class CommandConstantsTest {

    @Test
    void testTimingConstants() {
        assertEquals(0.0, CommandConstants.DEFAULT_WAIT_TIME, 
                "DEFAULT_WAIT_TIME should be 0.0");
        assertEquals(0.12, CommandConstants.SHORT_DRIVE_TIMEOUT, 
                "SHORT_DRIVE_TIMEOUT should be 0.12");
        assertEquals(0.15, CommandConstants.MEDIUM_DRIVE_TIMEOUT, 
                "MEDIUM_DRIVE_TIMEOUT should be 0.15");
        assertEquals(0.17, CommandConstants.LONG_DRIVE_TIMEOUT, 
                "LONG_DRIVE_TIMEOUT should be 0.17");
        assertEquals(0.35, CommandConstants.POST_SCORE_BACKWARD_TIMEOUT, 
                "POST_SCORE_BACKWARD_TIMEOUT should be 0.35");
        assertEquals(0.40, CommandConstants.POST_INTAKE_ARM_DELAY, 
                "POST_INTAKE_ARM_DELAY should be 0.40");
    }

    @Test
    void testSpeedConstants() {
        assertEquals(-0.5, CommandConstants.APPROACH_SCORE_SPEED, 
                "APPROACH_SCORE_SPEED should be -0.5");
        assertEquals(0.75, CommandConstants.PLACE_CORAL_FORWARD_SPEED, 
                "PLACE_CORAL_FORWARD_SPEED should be 0.75");
        assertEquals(-1.0, CommandConstants.POST_SCORE_BACKWARD_SPEED, 
                "POST_SCORE_BACKWARD_SPEED should be -1.0");
        assertEquals(0.0, CommandConstants.STOP_SPEED, 
                "STOP_SPEED should be 0.0");
    }

    @Test
    void testClimberVoltageConstants() {
        assertEquals(-1.75, CommandConstants.CLIMBER_SLOW_VOLTAGE, 
                "CLIMBER_SLOW_VOLTAGE should be -1.75");
        assertEquals(-3.5, CommandConstants.CLIMBER_FAST_VOLTAGE, 
                "CLIMBER_FAST_VOLTAGE should be -3.5");
        assertEquals(0.0, CommandConstants.CLIMBER_STOP_VOLTAGE, 
                "CLIMBER_STOP_VOLTAGE should be 0.0");
    }

    @Test
    void testConstantsAreAccessible() {
        // Verify all constants can be accessed
        assertNotNull(CommandConstants.DEFAULT_WAIT_TIME);
        assertNotNull(CommandConstants.SHORT_DRIVE_TIMEOUT);
        assertNotNull(CommandConstants.MEDIUM_DRIVE_TIMEOUT);
        assertNotNull(CommandConstants.LONG_DRIVE_TIMEOUT);
        assertNotNull(CommandConstants.POST_SCORE_BACKWARD_TIMEOUT);
        assertNotNull(CommandConstants.POST_INTAKE_ARM_DELAY);
        assertNotNull(CommandConstants.APPROACH_SCORE_SPEED);
        assertNotNull(CommandConstants.PLACE_CORAL_FORWARD_SPEED);
        assertNotNull(CommandConstants.POST_SCORE_BACKWARD_SPEED);
        assertNotNull(CommandConstants.STOP_SPEED);
        assertNotNull(CommandConstants.CLIMBER_SLOW_VOLTAGE);
        assertNotNull(CommandConstants.CLIMBER_FAST_VOLTAGE);
        assertNotNull(CommandConstants.CLIMBER_STOP_VOLTAGE);
    }
}
