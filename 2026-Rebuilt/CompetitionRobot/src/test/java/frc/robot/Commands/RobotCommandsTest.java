package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.ScoreSide;
import frc.robot.TestUtils;

/**
 * Unit tests for RobotCommands factory class.
 * Tests command creation, null validation, and basic functionality.
 */
class RobotCommandsTest {

    private RobotContext mockContext;

    @BeforeEach
    void setUp() {
        mockContext = TestUtils.createMockRobotContext();
    }

    @Test
    void testPrepareScoreCoralCommandWithNullContext() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.prepareScoreCoralCommand(null, ScoreLevel.L1, ScoreSide.Center);
        }, "Should throw NullPointerException for null context");
    }

    @Test
    void testPrepareScoreCoralCommandWithNullLevel() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.prepareScoreCoralCommand(mockContext, null, ScoreSide.Center);
        }, "Should throw NullPointerException for null level");
    }

    @Test
    void testPrepareScoreCoralCommandWithNullSide() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.prepareScoreCoralCommand(mockContext, ScoreLevel.L1, null);
        }, "Should throw NullPointerException for null side");
    }

    @Test
    void testPrepareScoreCoralCommandReturnsCommand() {
        Command command = RobotCommands.prepareScoreCoralCommand(
                mockContext, ScoreLevel.L2, ScoreSide.Left);
        assertNotNull(command, "prepareScoreCoralCommand should return a non-null command");
    }

    @Test
    void testPrepareScoreCoralRetryCommandWithNullContext() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.prepareScoreCoralRetryCommand(null);
        }, "Should throw NullPointerException for null context");
    }

    @Test
    void testPrepareScoreCoralRetryCommandReturnsCommand() {
        Command command = RobotCommands.prepareScoreCoralRetryCommand(mockContext);
        assertNotNull(command, "prepareScoreCoralRetryCommand should return a non-null command");
    }

    @Test
    void testScoreCoralCommandWithNullContext() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.scoreCoralCommand(null);
        }, "Should throw NullPointerException for null context");
    }

    @Test
    void testScoreCoralCommandReturnsCommand() {
        Command command = RobotCommands.scoreCoralCommand(mockContext);
        assertNotNull(command, "scoreCoralCommand should return a non-null command");
    }

    @Test
    void testPreIntakeCoralCommandWithNullContext() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.preIntakeCoralCommand(null);
        }, "Should throw NullPointerException for null context");
    }

    @Test
    void testPreIntakeCoralCommandReturnsCommand() {
        Command command = RobotCommands.preIntakeCoralCommand(mockContext);
        assertNotNull(command, "preIntakeCoralCommand should return a non-null command");
    }

    @Test
    void testIntakeCoralCommandWithNullContext() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.intakeCoralCommand(null);
        }, "Should throw NullPointerException for null context");
    }

    @Test
    void testIntakeCoralCommandReturnsCommand() {
        Command command = RobotCommands.intakeCoralCommand(mockContext);
        assertNotNull(command, "intakeCoralCommand should return a non-null command");
    }

    @Test
    void testPostIntakeCoralCommandWithNullContext() {
        assertThrows(NullPointerException.class, () -> {
            RobotCommands.postIntakeCoralCommand(null);
        }, "Should throw NullPointerException for null context");
    }

    @Test
    void testPostIntakeCoralCommandReturnsCommand() {
        Command command = RobotCommands.postIntakeCoralCommand(mockContext);
        assertNotNull(command, "postIntakeCoralCommand should return a non-null command");
    }

    @Test
    void testPrepareScoreCoralCommandWithInvalidLevel() {
        // This test would require creating a custom ScoreLevel enum value or using reflection
        // For now, we test that valid levels work
        for (ScoreLevel level : ScoreLevel.values()) {
            if (level != ScoreLevel.None) {
                Command command = RobotCommands.prepareScoreCoralCommand(
                        mockContext, level, ScoreSide.Center);
                assertNotNull(command, "Command should be created for level: " + level);
            }
        }
    }
}
