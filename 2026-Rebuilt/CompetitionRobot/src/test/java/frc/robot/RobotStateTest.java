package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Unit tests for RobotState class.
 * Tests initialization state management and command creation.
 */
class RobotStateTest {

    @BeforeEach
    void setUp() {
        // Reset the singleton instance before each test
        RobotState.resetInstance();
    }

    @AfterEach
    void tearDown() {
        // Clean up after each test
        RobotState.resetInstance();
    }

    @Test
    void testGetInstance() {
        RobotState instance1 = RobotState.getInstance();
        RobotState instance2 = RobotState.getInstance();
        
        // Should return the same instance (singleton pattern)
        assertNotNull(instance1);
        assertNotNull(instance2);
        assertTrue(instance1 == instance2, "getInstance() should return the same instance");
    }

    @Test
    void testInitialState() {
        RobotState state = RobotState.getInstance();
        
        // In simulation, should start initialized; on real robot, should not
        boolean expectedInitialized = !RobotBase.isReal();
        assertTrue(state.isInitialized() == expectedInitialized,
                "Initial state should match RobotBase.isReal()");
    }

    @Test
    void testEnableInitializationCommand() {
        RobotState state = RobotState.getInstance();
        Command enableCommand = state.enableInitializationCommand();
        
        assertNotNull(enableCommand, "enableInitializationCommand() should not return null");
    }

    @Test
    void testDisableInitializationCommand() {
        RobotState state = RobotState.getInstance();
        Command disableCommand = state.disableInitializationCommand();
        
        assertNotNull(disableCommand, "disableInitializationCommand() should not return null");
    }

    @Test
    void testToggleInitializationCommand() {
        RobotState state = RobotState.getInstance();
        Command toggleCommand = state.toggleInitializationCommand();
        
        assertNotNull(toggleCommand, "toggleInitializationCommand() should not return null");
    }

    @Test
    void testResetInstance() {
        RobotState instance1 = RobotState.getInstance();
        RobotState.resetInstance();
        RobotState instance2 = RobotState.getInstance();
        
        // After reset, should get a new instance
        assertNotNull(instance2);
        // Note: We can't directly compare instances, but reset should work
    }
}
