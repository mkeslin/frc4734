---
name: Architecture Improvements 2026
overview: Comprehensive architecture review and improvement recommendations for the 2026 FRC robot codebase, organized by priority with specific file references and actionable steps.
todos:
  - id: fix-elevator-follower
    content: Fix Elevator follower motor configuration (line 108) - either configure properly or remove
    status: completed
  - id: remove-commented-code
    content: Remove all commented code from Robot.java, RobotContainer.java, Elevator.java, and AutoCommandA.java
    status: completed
  - id: flatten-constants
    content: Refactor Constants.IDs nested class into separate constant classes (CANIds, DigitalInputIds, etc.)
    status: completed
  - id: state-machine-instance
    content: Convert StateMachine from static to instance-based class
    status: completed
  - id: split-robotcontainer
    content: Split RobotContainer into SubsystemFactory, BindingConfigurator, and AutoConfigurator classes
    status: pending
  - id: positiontracker-constructor
    content: Convert PositionTracker to use constructor injection instead of setters
    status: completed
  - id: command-organization
    content: Refactor RobotCommands to use RobotContext object or individual command classes
    status: completed
  - id: centralize-telemetry
    content: Create shared Telemetry class for NetworkTables usage
    status: pending
  - id: extract-magic-numbers
    content: Extract all magic numbers to appropriate constants files
    status: pending
---

# Architecture Improvements for 2026 FRC Robot Codebase

## Overview

This document outlines recommended improvements to enhance code maintainability, testability, and organization. Recommendations are organized by priority with specific file references.

## Completion Status

**Completed (6 tasks):**

- ✅ Fix Elevator Follower Configuration
- ✅ Remove Commented Code  
- ✅ Flatten Constants Structure
- ✅ Convert State Machine to Instance-Based
- ✅ Improve PositionTracker Design
- ✅ Better Command Organization

**Remaining (3 tasks):**

- ⏳ Split RobotContainer Responsibilities
- ⏳ Centralize NetworkTables Usage
- ⏳ Extract Magic Numbers to Constants

## High Priority Improvements

### 1. ✅ Fix Elevator Follower Configuration - COMPLETED

**File:** `src/main/java/frc/robot/Subsystems/Elevator.java` (line 108)

**Status:** ✅ **COMPLETED** - Follower motor is now properly configured using Phoenix 6 API with `MotorAlignmentValue.Aligned`.

**Changes Made:**

- Fixed neutral mode assignment (was setting left motor twice, now correctly sets right motor)
- Enabled follower configuration: `m_elevatorRightFollowerMotor.setControl(new Follower(m_elevatorLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned))`
- Added proper import for `MotorAlignmentValue`

### 2. ✅ Remove Commented Code - COMPLETED

**Files:** Multiple files contained large blocks of commented code

**Status:** ✅ **COMPLETED** - All commented code has been removed from:

- `src/main/java/frc/robot/Robot.java` - Removed ~22 lines of commented logging and initialization code
- `src/main/java/frc/robot/RobotContainer.java` - Removed ~100+ lines including commented subsystems, bindings, and methods
- `src/main/java/frc/robot/Subsystems/Elevator.java` - Removed commented simulation, telemetry, and method code
- `src/main/java/frc/robot/Auto/AutoCommandA.java` - Removed commented command methods and inline comments
- Also removed 3 unused imports from RobotContainer.java

**Result:** Codebase is now cleaner and easier to maintain. All history is preserved in version control.

### 3. ✅ Flatten Constants Structure - COMPLETED

**File:** `src/main/java/frc/robot/Constants/Constants.java`

**Status:** ✅ **COMPLETED** - Constants structure has been flattened into separate classes.

**Changes Made:**

- Created `CANIds.java` - Contains all CAN device IDs (ELEVATOR_LEFT, ELEVATOR_RIGHT, ARM, SIDE_TO_SIDE, CLIMBER, ALGAE_INTAKE, LIGHTS)
- Created `DigitalInputIds.java` - Contains digital input sensor IDs (CORAL_TRAY_SENSOR, CORAL_ARM_SENSOR)
- Created `VisionConstants.java` - Contains vision-related constants (APRILTAG_PIPELINE)
- Updated `Constants.java` - Now a utility class with documentation pointing to new constant files
- Updated all references across codebase (RobotContainer, Elevator, Arm, SideToSide, Climber, Lights, AlgaeIntake)

**Naming Improvements:**

- Removed redundant `_ID` suffix from CAN IDs (e.g., `ELEVATOR_LEFT_ID` → `ELEVATOR_LEFT`)
- Renamed `APRILTAGPIPELINE` → `APRILTAG_PIPELINE` for consistency

### 4. ✅ Convert State Machine to Instance-Based - COMPLETED

**File:** `src/main/java/frc/robot/State/StateMachine.java`

**Status:** ✅ **COMPLETED** - StateMachine is now instance-based with proper dependency injection.

**Changes Made:**

- Removed `static` keyword from all fields (`m_stateMap`, `m_currentStateName`, `m_allStates`)
- Removed `static` keyword from all methods (`Load()`, `GetCurrentState()`, `CanTransition()`)
- Added constructor that calls `load()` (renamed from `Load()`)
- Made `load()` private method
- Renamed methods to camelCase: `getCurrentState()`, `canTransition()`
- Added comprehensive JavaDoc
- Updated `RobotContainer` to create `StateMachine` instance
- Updated all `RobotCommands` methods to accept `StateMachine` as parameter
- Updated `AutoCommandA` to accept and use `StateMachine` instance

**Benefits Achieved:**

- Better testability - can create multiple instances for testing
- Thread safety - each instance has its own state
- Dependency injection - follows DI principles
- No static state - avoids global mutable state issues

### 5. Split RobotContainer Responsibilities

**File:** `src/main/java/frc/robot/RobotContainer.java` (367 lines)

**Issue:** Single class handles too many responsibilities:

- Subsystem creation
- Controller bindings (3 separate methods)
- Auto configuration
- Position tracker setup
- State machine loading

**Action:** Create separate classes:

- `SubsystemFactory.java` - Factory for creating all subsystems
- `BindingConfigurator.java` - Handles all controller bindings (consolidate `configureMechanismBindings`, `configureDriveBindings`, `configureArcadeBindings`)
- `AutoConfigurator.java` - Handles autonomous routine setup
- Keep `RobotContainer` as coordinator that uses these classes

## Medium Priority Improvements

### 6. ✅ Improve PositionTracker Design - COMPLETED

**File:** `src/main/java/frc/robot/PositionTracker.java`

**Status:** ✅ **COMPLETED** - PositionTracker now uses constructor injection with all suppliers required at creation time.

**Changes Made:**

- Converted to constructor injection - all suppliers are required parameters
- Made all supplier fields `final` to prevent modification after construction
- Removed all setter methods (`setElevatorPositionSupplier`, etc.)
- Updated all subsystem constructors to remove PositionTracker parameter (no longer needed)
- Updated RobotContainer to create PositionTracker after subsystems with method references
- Added `setPositionTracker()` method to all subsystems to share the same instance
- Added null checks in telemetry publishing methods for safety
- Added comprehensive JavaDoc explaining constructor parameters and inverted sensor logic

**Benefits:**

- Prevents null pointer exceptions - all suppliers guaranteed at construction
- All subsystems share the same PositionTracker instance with real suppliers
- Better encapsulation - no public setters, dependencies are explicit
- Improved documentation - JavaDoc explains inverted sensor logic

### 7. ✅ Better Command Organization - COMPLETED

**File:** `src/main/java/frc/robot/Commands/RobotCommands.java`

**Status:** ✅ **COMPLETED** - RobotCommands now uses RobotContext to reduce parameter count and improve readability.

**Changes Made:**

- Created `RobotContext.java` - New class containing all common dependencies:
  - `StateMachine`, `PositionTracker`, `CommandSwerveDrivetrain`, `Elevator`, `Arm`, `SideToSide`, `Lights`, `Limelight`
  - All fields are `final` for immutability
  - Comprehensive JavaDoc documentation
- Refactored all `RobotCommands` methods to accept `RobotContext` instead of 8-9 individual parameters:
  - `prepareScoreCoralCommand`: 10 params → 3 params (`RobotContext`, `ScoreLevel`, `ScoreSide`)
  - `prepareScoreCoralRetryCommand`: 8 params → 1 param (`RobotContext`)
  - `scoreCoralCommand`: 6 params → 1 param (`RobotContext`)
  - `preIntakeCoralCommand`: 6 params → 1 param (`RobotContext`)
  - `intakeCoralCommand`: 6 params → 1 param (`RobotContext`)
  - `postIntakeCoralCommand`: 6 params → 1 param (`RobotContext`)
- Updated `RobotContainer` to create `RobotContext` instance and pass it to all command calls
- Updated `AutoCommandA` to use `RobotContext` instead of individual subsystem parameters
- Added JavaDoc to all command methods

**Benefits Achieved:**

- Improved readability - methods have dramatically fewer parameters
- Easier maintenance - dependencies centralized in one place
- Better testability - can create RobotContext with mock dependencies
- Reduced errors - fewer parameters to pass incorrectly
- Cleaner code - call sites are much simpler and easier to understand

### 8. Centralize NetworkTables Usage

**Files:** Multiple subsystem files create their own NetworkTable instances

**Issue:** Each subsystem creates its own NetworkTable instance, leading to scattered telemetry.

**Action:** Create a shared `Telemetry` class:

```java
public class Telemetry {
    private static final NetworkTable mechanismsTable = 
        NetworkTableInstance.getDefault().getTable("Mechanisms");
    
    public static DoublePublisher createPublisher(String topic) {
        return mechanismsTable.getDoubleTopic(topic).publish();
    }
}
```

### 9. Extract Magic Numbers to Constants

**Files:** Multiple files contain hard-coded values

**Examples:**

- `RobotContainer.java`: Timeouts (0.12, 0.15, 0.17, 0.35), speeds (-0.5, 0.75, -1.0)
- `AutoCommandA.java`: Wait times (0.0, 0.1, 0.35, 15.0), speeds (-0.6, 0.75, -1.0)
- `StateMachine.java`: Console output (should use proper logging)

**Action:** Create constants files:

- `RobotContainerConstants.java` - For command timeouts and speeds
- `AutoConstants.java` - For autonomous timing and speeds
- Use WPILib's logging framework instead of `System.out.println`

## Low Priority Improvements

### 10. Consider Dependency Injection Framework

**Current:** Everything created in `RobotContainer` constructor

**Action:** Consider lightweight DI solutions or at least organize creation with factory methods. This is optional but would improve testability.

### 11. Improve Error Handling

**File:** `src/main/java/frc/robot/Auto/AutoCommandA.java` (lines 46-48, 89-90, etc.)

**Issue:** Exceptions are caught but only reported to DriverStation. No recovery strategy.

**Action:** Add proper error handling:

- Validate paths exist before use
- Provide fallback auto routines
- Log errors properly using WPILib logging

### 12. Replace AutoManager Singleton

**File:** `src/main/java/frc/robot/Auto/AutoManager.java`

**Issue:** Singleton pattern makes testing difficult.

**Action:** Convert to instance-based and pass as dependency through `RobotContainer`.

### 13. Improve GlobalStates Design

**File:** `src/main/java/frc/robot/GlobalStates.java`

**Issue:** Enum with mutable state is unusual design pattern.

**Action:** Consider:

- `RobotState` class instead of enum
- WPILib's built-in state management
- Or keep as-is if it works well for your use case

### 14. Extract Complex Command Sequences

**File:** `src/main/java/frc/robot/RobotContainer.java` (lines 188-263)

**Issue:** Complex command chains in `configureArcadeBindings()` are hard to read and maintain.

**Action:** Extract complex sequences into named methods:

```java
private Command createScoreSequence(ScoreLevel level, ScoreSide side) {
    // Extract the complex command logic
}
```

### 15. Leverage WPILib 2026 Features

**Action:** Review and adopt:

- New command composition features
- Improved logging framework (AdvantageKit or WPILib logging)
- Better simulation support
- New path planning features

## Implementation Order

1. **✅ Week 1:** High Priority items 1-3 (Quick wins) - **COMPLETED**

   - ✅ Fix Elevator Follower Configuration
   - ✅ Remove Commented Code
   - ✅ Flatten Constants Structure

2. **✅ Week 2:** High Priority items 4-5 (Architectural changes) - **PARTIALLY COMPLETED**

   - ✅ Convert State Machine to Instance-Based
   - ⏳ Split RobotContainer Responsibilities

3. **✅ Week 3:** Medium Priority items 6-9 (Code quality) - **PARTIALLY COMPLETED**

   - ✅ Improve PositionTracker Design
   - ✅ Better Command Organization
   - ⏳ Centralize NetworkTables Usage
   - ⏳ Extract Magic Numbers to Constants

4. **Week 4+:** Low Priority items as needed (Polish)

## Notes

- Test thoroughly after each change
- Use version control branches for major refactoring
- Consider team review for architectural changes
- Document any new patterns for future reference
