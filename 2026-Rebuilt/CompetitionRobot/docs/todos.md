# TODO List - Competition Robot Codebase

This document contains all TODO items found in the codebase, organized by priority. Each item includes the file location, description, and what needs to be done.

## Priority Levels
- **HIGH**: Critical for robot functionality - blocks core features
- **MEDIUM**: Important for safety/performance - should be completed before competition
- **LOW**: Nice to have - can be deferred if needed

---

## HIGH PRIORITY

### 1. Field Constants and Positions

#### 1.1 Update 2026 Field Hub Position
- **File**: `src/main/java/frc/robot/Auto/commands/AutoConstants.java:40`
- **Current**: Placeholder hub position at (8.27, 4.11)
- **Action Required**: 
  - Measure actual 2026 field hub center position
  - Update `HUB_POSE` constant with accurate coordinates
  - Verify rotation angle is correct
- **Impact**: Autonomous routines that aim at the hub will miss if this is incorrect

#### 1.2 Update 2026 Field Layout Constant Name
- **File**: `src/main/java/frc/robot/PathPlanner/AllianceUtils.java:167, 179`
- **Current**: Using `AprilTagFields.kDefaultField` as placeholder
- **Action Required**:
  - Determine the actual 2026 field layout constant name (e.g., `AprilTagFields.k2026RebuiltAndyMark`)
  - Update `getFieldLayout()` method to use the correct constant
  - Update `getFieldLayoutName()` method for logging/debugging
- **Impact**: Vision-based odometry and AprilTag detection may not work correctly without the proper field layout

#### 1.3 Add 2026 Field Landmarks
- **Files**: 
  - `src/main/java/frc/robot/PathPlanner/BlueLandmarks.java:12`
  - `src/main/java/frc/robot/PathPlanner/Landmarks.java:43`
- **Current**: Only starting positions defined
- **Action Required**:
  - Add hub position landmark to `BlueLandmarks.java`
  - Add tower position landmark to `BlueLandmarks.java`
  - Add any other important field elements (scoring zones, etc.)
  - Add corresponding methods to `Landmarks.java` for alliance-aware access
- **Impact**: Path planning and autonomous routines cannot reference important field elements

---

### 2. Constant Tuning and Configuration

#### 2.1 Tune Shooter Speed Values
- **File**: `src/main/java/frc/robot/Constants/ShooterConstants.java:14-15`
- **Current**: 
  - FORWARD: 0.5 (placeholder)
  - REVERSE: -0.3 (placeholder)
- **Action Required**:
  - Test and tune forward speed for accurate shooting at various distances
  - Test and tune reverse speed for clearing jams
  - Update enum values with tested values
- **Impact**: Shooter will not shoot accurately without proper speed tuning

#### 2.2 Tune Feeder Speed Values
- **File**: `src/main/java/frc/robot/Constants/FeederConstants.java:14-15`
- **Current**:
  - FORWARD: 0.3 (placeholder)
  - REVERSE: -0.3 (placeholder)
- **Action Required**:
  - Test forward speed to ensure reliable ball feeding to shooter
  - Test reverse speed for clearing jams
  - Update enum values with tested values
- **Impact**: Feeder may not reliably move balls to shooter or clear jams

#### 2.3 Tune Floor Conveyor Speed Values
- **File**: `src/main/java/frc/robot/Constants/FloorConstants.java:14-15`
- **Current**:
  - FORWARD: 0.3 (placeholder)
  - REVERSE: -0.3 (placeholder)
- **Action Required**:
  - Test forward speed to ensure reliable ball movement to feeder
  - Test reverse speed for clearing jams
  - Update enum values with tested values
- **Impact**: Floor conveyor may not reliably move balls or clear jams

#### 2.4 Update Intake Deployed Position
- **File**: `src/main/java/frc/robot/Constants/IntakeConstants.java:14`
- **Current**: DEPLOYED = 1.0 rotations (placeholder)
- **Action Required**:
  - Measure actual deployed position in rotations
  - Update enum value with measured position
- **Impact**: Intake may not deploy to correct position

#### 2.5 Tune Intake Deploy Motor PID/MotionMagic Constants
- **File**: `src/main/java/frc/robot/Constants/IntakeConstants.java:42`
- **Current**: Has placeholder values (kP=3.8, kI=0.0, kD=0.1, kS=0.25, kV=0.12, kA=0.01, kG=-0.2)
- **Action Required**:
  - Use SysId or manual tuning to determine correct PID values
  - Tune MotionMagic cruise velocity, acceleration, and jerk
  - Update all constants with tuned values
- **Impact**: Intake deployment may be slow, inaccurate, or oscillate

#### 2.6 Tune Climber PID and Feedforward Constants
- **File**: `src/main/java/frc/robot/Constants/ClimberConstants.java:31-40`
- **Current**: All values marked as TODO with placeholder values:
  - kP = 10
  - kI = 0
  - kD = 0
  - kS = 0.017964
  - kG = 0.321192
  - kV = 0.876084
  - kA = 0.206676
  - MAX_VELOCITY_METERS_PER_SECOND = 8
  - MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4
- **Action Required**:
  - Use SysId to characterize climber motor
  - Tune PID values for accurate position control
  - Tune feedforward values (kS, kG, kV, kA) for smooth motion
  - Determine appropriate max velocity and acceleration limits
  - Update all constants with tuned values
- **Impact**: Climber will not move accurately or smoothly, may damage mechanism

---

### 3. Missing Constant References in Commands

#### 3.1 Replace Hardcoded Intake Speed in CmdIntakeOn
- **File**: `src/main/java/frc/robot/Auto/commands/CmdIntakeOn.java:45`
- **Current**: Using hardcoded value 0.5
- **Action Required**:
  - Replace with `IntakeConstants.IntakeSpeed.IN.value`
  - Or use appropriate constant from IntakeConstants
- **Impact**: Intake speed may not match tuned values, causing inconsistent behavior

#### 3.2 Replace Hardcoded Intake Speed in CmdIntakeUntilCount
- **File**: `src/main/java/frc/robot/Auto/commands/CmdIntakeUntilCount.java:69`
- **Current**: Using hardcoded value 0.5
- **Action Required**:
  - Replace with `IntakeConstants.IntakeSpeed.IN.value`
- **Impact**: Intake speed may not match tuned values

#### 3.3 Replace Hardcoded Feeder Speed in CmdShootForTime
- **File**: `src/main/java/frc/robot/Auto/commands/CmdShootForTime.java:84`
- **Current**: Using hardcoded value 0.5
- **Action Required**:
  - Replace with `FeederConstants.FeederSpeed.FORWARD.value`
- **Impact**: Feeder speed may not match tuned values, affecting shot consistency

---

## MEDIUM PRIORITY

### 4. Safety Coordination Checks

#### 4.1 Add Shooter Safety Coordination Checks
- **File**: `src/main/java/frc/robot/Subsystems/Shooter.java:123`
- **Current**: `canRun()` method is empty placeholder
- **Action Required**:
  - Determine safety conditions (e.g., require feeder ready, check ball presence)
  - Implement checks in `canRun()` method
  - Add corresponding constants to `ShooterConstants.java` (see 4.4)
- **Impact**: Shooter may run when it shouldn't, potentially causing damage or unsafe conditions

#### 4.2 Add Feeder Safety Coordination Checks
- **File**: `src/main/java/frc/robot/Subsystems/Feeder.java:105`
- **Current**: `canRun()` method is empty placeholder
- **Action Required**:
  - Determine safety conditions (e.g., require shooter ready, coordinate with Floor)
  - Implement checks in `canRun()` method
  - Add corresponding constants to `FeederConstants.java` (see 4.5)
- **Impact**: Feeder may run when it shouldn't, potentially causing jams or unsafe conditions

#### 4.3 Add Floor Conveyor Safety Coordination Checks
- **File**: `src/main/java/frc/robot/Subsystems/Floor.java:105`
- **Current**: `canRun()` method is empty placeholder
- **Action Required**:
  - Determine safety conditions (e.g., require shooter ready, stop when hopper full)
  - Implement checks in `canRun()` method
  - Add corresponding constants to `FloorConstants.java` (see 4.6)
- **Impact**: Floor conveyor may run when it shouldn't, potentially causing jams or unsafe conditions

#### 4.4 Add Shooter Safety Constants
- **File**: `src/main/java/frc/robot/Constants/ShooterConstants.java:25`
- **Current**: Placeholder comment with examples
- **Action Required**:
  - Define boolean flags for safety requirements (e.g., `REQUIRE_FEEDER_READY`, `CHECK_BALL_PRESENCE`)
  - Add constants based on determined safety conditions
- **Impact**: Safety coordination cannot be configured without these constants

#### 4.5 Add Feeder Safety Constants
- **File**: `src/main/java/frc/robot/Constants/FeederConstants.java:25`
- **Current**: Placeholder comment with examples
- **Action Required**:
  - Define boolean flags for safety requirements (e.g., `REQUIRE_SHOOTER_READY`, `COORDINATE_WITH_FLOOR`)
  - Add constants based on determined safety conditions
- **Impact**: Safety coordination cannot be configured without these constants

#### 4.6 Add Floor Conveyor Safety Constants
- **File**: `src/main/java/frc/robot/Constants/FloorConstants.java:25`
- **Current**: Placeholder comment with examples
- **Action Required**:
  - Define boolean flags for safety requirements (e.g., `REQUIRE_SHOOTER_READY`, `STOP_WHEN_HOPPER_FULL`)
  - Add constants based on determined safety conditions
- **Impact**: Safety coordination cannot be configured without these constants

---

### 5. Shot Model Integration

#### 5.1 Integrate Shot Model/Calculator
- **File**: `src/main/java/frc/robot/Auto/commands/CmdSetShotMode.java:46`
- **Current**: Placeholder command that completes immediately
- **Action Required**:
  - Design or integrate shot model/calculator system
  - Implement logic to set shot mode and calculate RPM based on distance
  - Update command to actually configure shooter based on shot mode
- **Impact**: Cannot use different shot modes (AUTO_SHOT, SAFE_SHOT) for different distances

---

### 6. Climber Qualification Check

#### 6.1 Add Climber L1 Qualification Check
- **File**: `src/main/java/frc/robot/Auto/commands/CmdClimbL1.java:82`
- **Current**: Only checks position, no qualification check
- **Action Required**:
  - Determine what "L1 qualified" means (e.g., climber hooks engaged, mechanism locked)
  - Add `isL1Qualified()` method to Climber subsystem if needed
  - Update command to check qualification before completing
- **Impact**: Command may complete before climber is actually ready, causing failed climbs

---

## LOW PRIORITY

### 7. Simulation Support

#### 7.1 Add Shooter Simulation
- **File**: `src/main/java/frc/robot/Subsystems/Shooter.java:94`
- **Action Required**:
  - Implement `simulationPeriodic()` method if simulation is needed
  - Add motor simulation models
- **Impact**: Cannot test shooter in simulation environment

#### 7.2 Add Feeder Simulation
- **File**: `src/main/java/frc/robot/Subsystems/Feeder.java:76`
- **Action Required**:
  - Implement `simulationPeriodic()` method if simulation is needed
  - Add motor simulation models
- **Impact**: Cannot test feeder in simulation environment

#### 7.3 Add Floor Conveyor Simulation
- **File**: `src/main/java/frc/robot/Subsystems/Floor.java:76`
- **Action Required**:
  - Implement `simulationPeriodic()` method if simulation is needed
  - Add motor simulation models
- **Impact**: Cannot test floor conveyor in simulation environment

#### 7.4 Add DeployableIntake Simulation
- **File**: `src/main/java/frc/robot/Subsystems/DeployableIntake.java:142`
- **Action Required**:
  - Implement `simulationPeriodic()` method if simulation is needed
  - Add motor simulation models for both deploy and intake motors
- **Impact**: Cannot test intake in simulation environment

---

### 8. Additional Controller Bindings

#### 8.1 Add 2026 Drive Controller Bindings
- **File**: `src/main/java/frc/robot/BindingConfigurator.java:100`
- **Current**: Only climber bindings configured
- **Action Required**:
  - Add bindings for other 2026 game-specific features as needed
  - Configure any additional drive controller buttons/triggers
- **Impact**: Missing functionality that could be controlled from drive controller

---

## Summary

**Total TODOs**: 39

**By Priority**:
- **HIGH**: 15 items (critical for functionality)
- **MEDIUM**: 7 items (important for safety/performance)
- **LOW**: 5 items (nice to have)

**By Category**:
- Field constants/positions: 3
- Constant tuning: 6
- Missing constant references: 3
- Safety coordination: 6
- Shot model integration: 1
- Climber qualification: 1
- Simulation support: 4
- Controller bindings: 1

---

## Recommended Order of Completion

1. **Field Constants** (Items 1.1-1.3) - Required for autonomous to work
2. **Constant Tuning** (Items 2.1-2.6) - Required for subsystems to function correctly
3. **Missing Constant References** (Items 3.1-3.3) - Quick fixes to use tuned values
4. **Safety Coordination** (Items 4.1-4.6) - Important before competition
5. **Shot Model Integration** (Item 5.1) - Enhances autonomous capabilities
6. **Climber Qualification** (Item 6.1) - Improves climb reliability
7. **Simulation Support** (Items 7.1-7.4) - Only if simulation testing is needed
8. **Controller Bindings** (Item 8.1) - Add as features are developed

---

*Last Updated: Generated from codebase scan*
*Total Files Scanned: 48 Java files*
