package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.Mockito.mock;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

/**
 * Unit tests for CenterToReefCommand class.
 * Tests constructor validation and basic command creation.
 */
class CenterToReefCommandTest {

    private PhotonVision mockPhotonVision;
    private CommandSwerveDrivetrain mockDrivetrain;
    private CommandXboxController mockController;

    @BeforeEach
    void setUp() {
        // Create mocks using Mockito
        mockPhotonVision = mock(PhotonVision.class);
        mockDrivetrain = mock(CommandSwerveDrivetrain.class);
        mockController = mock(CommandXboxController.class);
    }

    @Test
    void testConstructorWithNullPhotonVision() {
        assertThrows(NullPointerException.class, () -> {
            new CenterToReefCommand(null, mockDrivetrain, mockController, 3.0);
        }, "Should throw NullPointerException for null PhotonVision");
    }

    @Test
    void testConstructorWithNullDrivetrain() {
        assertThrows(NullPointerException.class, () -> {
            new CenterToReefCommand(mockPhotonVision, null, mockController, 3.0);
        }, "Should throw NullPointerException for null CommandSwerveDrivetrain");
    }

    @Test
    void testConstructorWithInvalidTimeout() {
        assertThrows(IllegalArgumentException.class, () -> {
            new CenterToReefCommand(mockPhotonVision, mockDrivetrain, mockController, 0.0);
        }, "Should throw IllegalArgumentException for timeout <= 0");
        
        assertThrows(IllegalArgumentException.class, () -> {
            new CenterToReefCommand(mockPhotonVision, mockDrivetrain, mockController, -1.0);
        }, "Should throw IllegalArgumentException for negative timeout");
    }

    @Test
    void testConstructorWithValidParameters() {
        CenterToReefCommand command = new CenterToReefCommand(
                mockPhotonVision, mockDrivetrain, mockController, 3.0);
        
        assertNotNull(command, "Command should be created successfully with valid parameters");
    }

    @Test
    void testConstructorWithNullController() {
        // Controller is optional, so null should be allowed
        CenterToReefCommand command = new CenterToReefCommand(
                mockPhotonVision, mockDrivetrain, null, 3.0);
        
        assertNotNull(command, "Command should be created successfully with null controller");
    }
}
