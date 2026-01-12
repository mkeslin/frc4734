---
name: FRC Software Best Practices Implementation
overview: Implement comprehensive software best practices used by top FRC teams, including AdvantageKit logging/replay, enhanced testing, code quality tools, error recovery strategies, and performance monitoring.
todos:
  - id: advantagekit_setup
    content: Add AdvantageKit dependency and create RobotLogger wrapper class
    status: pending
  - id: advantagekit_migration
    content: Migrate DataLogManager calls to AdvantageKit logging in all subsystems
    status: pending
  - id: simulation_tests
    content: Create SimulationTestBase and add integration tests for command sequences
    status: pending
  - id: autonomous_tests
    content: Add simulation tests for autonomous routines and path validation
    status: pending
  - id: spotless_setup
    content: Configure Spotless code formatter in build.gradle
    status: pending
  - id: github_actions
    content: Set up GitHub Actions CI/CD pipeline for automated testing
    status: pending
  - id: error_handler
    content: Create ErrorHandler utility and retry logic for critical operations
    status: pending
  - id: health_monitor
    content: Implement SubsystemHealthMonitor for tracking subsystem status
    status: pending
  - id: performance_monitor
    content: Add PerformanceMonitor class for loop timing and CAN utilization tracking
    status: pending
  - id: alliance_utils
    content: Create AllianceUtils helper class for alliance-aware coordinate transformations
    status: pending
---

# FRC Software Best Practices Implementation Plan

## Overview

This plan implements best practices used by top FRC teams (254, 1678, 6328, etc.) to improve code quality, reliability, debugging capabilities, and maintainability.

## 1. Logging & Replay System (AdvantageKit)

**Why**: AdvantageKit is the industry standard for FRC logging and replay. It enables:

- High-frequency data logging (1000+ Hz)
- Replay capabilities for debugging match issues
- AdvantageScope integration for data visualization
- Automatic logging of all robot state

**Implementation**:

- Add AdvantageKit dependency to `build.gradle`
- Create `RobotLogger.java` wrapper class
- Replace `DataLogManager` calls with AdvantageKit logging
- Add periodic logging in subsystems (pose, mechanism positions, sensor values)
- Configure logging rates (fast loop for drivetrain, slower for mechanisms)
- Set up AdvantageScope for data analysis

**Files to modify**:

- `build.gradle` - Add AdvantageKit dependency
- Create `src/main/java/frc/robot/Logging/RobotLogger.java`
- Update all subsystems to use AdvantageKit logging
- Update `Robot.java` to initialize AdvantageKit

**Benefits**: Match replay, better debugging, data-driven tuning

## 2. Enhanced Testing & Simulation

**Current state**: You have basic unit tests. Top teams add:

- Integration tests for command sequences
- Simulation tests for autonomous routines
- Hardware-in-the-loop (HIL) testing
- Vision pipeline testing

**Implementation**:

- Create `SimulationTestBase.java` for common simulation setup
- Add integration tests for command chains (e.g., `IntakeToScoreTest.java`)
- Create `AutonomousRoutineTest.java` to validate auto paths
- Add vision simulation tests using `PhotonCameraSim`
- Create `SwerveDrivetrainSimTest.java` for drivetrain validation
- Add test coverage reporting (JaCoCo)

**Files to create**:

- `src/test/java/frc/robot/SimulationTestBase.java`
- `src/test/java/frc/robot/Commands/Integration/IntakeToScoreTest.java`
- `src/test/java/frc/robot/Auto/AutonomousRoutineTest.java`
- `src/test/java/frc/robot/Subsystems/Vision/PhotonVisionSimTest.java`

**Benefits**: Catch bugs before competition, validate autonomous routines

## 3. Code Quality Tools

**Implementation**:

- **Spotless** (code formatter): Auto-format code on commit
- **Checkstyle** or **PMD**: Static code analysis
- **GitHub Actions CI/CD**: Automated testing on every commit
- **Pre-commit hooks**: Run tests and formatting before commit

**Configuration**:

- Add Spotless plugin to `build.gradle`
- Create `.spotless/java.prettierrc` configuration
- Create `.github/workflows/ci.yml` for GitHub Actions
- Add `checkstyle.xml` configuration
- Create `.git/hooks/pre-commit` script

**Files to create**:

- `.github/workflows/ci.yml`
- `config/checkstyle/checkstyle.xml`
- `.spotless/java.prettierrc`
- Update `build.gradle` with Spotless and Checkstyle plugins

**Benefits**: Consistent code style, catch issues early, automated quality checks

## 4. Error Handling & Recovery Strategies

**Current state**: You log errors but don't have recovery strategies.

**Implementation**:

- Create `ErrorHandler.java` utility class
- Add retry logic for critical operations (vision, CAN bus)
- Implement fallback behaviors (e.g., if vision fails, use odometry)
- Add health monitoring for subsystems
- Create `SubsystemHealthMonitor.java` to track subsystem status
- Add circuit breakers for unreliable operations

**Error Recovery Patterns**:

- Vision failures → Fall back to odometry-based positioning
- CAN timeout → Retry with exponential backoff
- Sensor failure → Use alternative sensor or estimated value
- Path following failure → Stop and report error

**Files to create**:

- `src/main/java/frc/robot/Utils/ErrorHandler.java`
- `src/main/java/frc/robot/Utils/RetryUtil.java`
- `src/main/java/frc/robot/Monitoring/SubsystemHealthMonitor.java`
- Update commands to use error recovery

**Benefits**: Robot continues operating despite failures, better match reliability

## 5. Performance Monitoring

**Implementation**:

- Add loop timing monitoring (ensure 20ms loop)
- Monitor CAN bus utilization
- Track command execution times
- Add performance metrics to AdvantageKit logs
- Create `PerformanceMonitor.java` class

**Metrics to track**:

- Main loop period (should be ~20ms)
- CAN bus utilization percentage
- Command scheduler overhead
- Vision processing time
- NetworkTables update frequency

**Files to create**:

- `src/main/java/frc/robot/Monitoring/PerformanceMonitor.java`
- Update `Robot.java` to track loop timing
- Add CAN utilization monitoring

**Benefits**: Identify performance bottlenecks, ensure real-time performance

## 6. Additional Best Practices

### 6.1 Alliance-Aware Field Coordinates

- Ensure all pose calculations handle alliance color correctly
- Add `AllianceUtils.java` helper class
- Verify PhotonVision handles alliance flipping

### 6.2 Defensive Programming

- Add input validation to all public methods
- Use `Objects.requireNonNull()` consistently (you already do this well)
- Add bounds checking for all setpoints
- Validate sensor readings before use

### 6.3 Documentation

- Add JavaDoc to all public APIs
- Create architecture diagrams (use Mermaid)
- Document tuning procedures
- Create troubleshooting guides

### 6.4 Configuration Management

- Externalize tunable parameters to NetworkTables
- Use `TunableNumber` from AdvantageKit for live tuning
- Create tuning dashboard in Shuffleboard

## Implementation Priority

**Phase 1 (High Impact, Low Effort)**:

1. Code Quality Tools (Spotless, CI/CD)
2. Performance Monitoring
3. Enhanced Error Handling

**Phase 2 (High Impact, Medium Effort)**:

4. AdvantageKit Integration
5. Enhanced Testing

**Phase 3 (Medium Impact, Ongoing)**:

6. Documentation improvements
7. Configuration management
8. Alliance-aware utilities

## Success Metrics

- Code coverage > 60%
- All tests pass in CI
- Zero unhandled exceptions in matches
- Loop timing consistently < 25ms
- Match replay available for all matches
- Code automatically formatted on commit

## Notes

- AdvantageKit requires WPILib 2024+ (you're on 2026, so compatible)
- Some tools (Spotless, Checkstyle) can be added incrementally
- Performance monitoring should be lightweight to not impact robot performance
- Error recovery should be tested in simulation first
