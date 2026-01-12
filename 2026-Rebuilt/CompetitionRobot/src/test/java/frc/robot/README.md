# Unit Test Infrastructure

This directory contains unit tests for the FRC robot codebase.

## Test Structure

Tests are organized to mirror the main source structure:
- `frc.robot` - Tests for core robot classes
- `frc.robot.Commands` - Tests for command classes
- `frc.robot.Constants` - Tests for constants classes

## Running Tests

### From Command Line
```bash
# Run all tests (tests are skipped during normal builds)
./gradlew test

# Run specific test class
./gradlew test --tests RobotStateTest

# Run tests with verbose output
./gradlew test --info

# Run tests as part of build (optional)
./gradlew build -PrunTests

# Note: Tests are automatically skipped during ./gradlew build or ./gradlew deploy
# They will only run when explicitly invoked with ./gradlew test
```

### From VS Code
1. Open the test file
2. Click the "Run Test" button above test methods
3. Or use the Test Explorer view

## Test Utilities

### TestUtils.java
Provides helper methods for creating test fixtures:
- `createMockPositionTracker()` - Creates a PositionTracker with default suppliers
- `createMockRobotContext()` - Creates a minimal RobotContext for testing
- `createDefaultPose3dSupplier()` - Creates a default Pose3d supplier

## Writing Tests

### Example Test Structure
```java
class MyClassTest {
    @BeforeEach
    void setUp() {
        // Setup test fixtures
    }

    @AfterEach
    void tearDown() {
        // Cleanup after tests
    }

    @Test
    void testMethodName() {
        // Arrange
        // Act
        // Assert
    }
}
```

## Testing Guidelines

1. **Test Isolation**: Each test should be independent and not rely on other tests
2. **Use @BeforeEach/@AfterEach**: For setup and cleanup
3. **Test Null Safety**: Verify null parameter validation
4. **Test Edge Cases**: Test boundary conditions and error cases
5. **Use Descriptive Names**: Test method names should describe what they test

## Dependencies

- **JUnit 5**: Testing framework (already configured)
- **Mockito**: For creating mock objects (added in build.gradle)

## Notes

- Tests run in simulation mode by default
- Some tests may require WPILib simulation to be enabled
- Hardware-dependent tests should use mocking or simulation
