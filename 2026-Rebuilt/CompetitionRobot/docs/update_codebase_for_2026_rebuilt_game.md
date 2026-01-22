---
name: Update Codebase for 2026 REBUILT Game
overview: Update all 2025 Reefscape game references to 2026 REBUILT game elements, including field layout, AprilTags, scoring elements (Coral→FUEL, Reef→HUB), command names, and subsystem references.
todos:
  - id: update-field-layout
    content: Update AprilTag field layout references from k2025ReefscapeAndyMark to k2026RebuiltAndyMark in PhotonVision.java and CenterToReefCommand.java
    status: pending
  - id: update-apriltag-ids
    content: Update AprilTag ID constants in BlueAprilTags.java, RedAprilTags.java, and AprilTags.java to match 2026 field layout (IDs 1-32)
    status: pending
  - id: update-apriltag-methods
    content: "Rename AprilTag helper methods: OurSpeakerRight/Left → OurHubRight/Left, OurAmp → OurOutpost, OurStage1/2/3 → OurTowerLow/Mid/High"
    status: pending
  - id: rename-center-to-reef
    content: Rename CenterToReefCommand.java to CenterToHubCommand.java and update AprilTag target logic for HUB tags (2-5, 8-11, 18-21, 24-27)
    status: pending
  - id: rename-center-to-processer
    content: Rename CenterToProcesserCommand.java to CenterToOutpostCommand.java (or CenterToDepotCommand.java after team confirmation)
    status: pending
  - id: rename-robot-commands
    content: "Rename all Coral methods in RobotCommands.java: prepareScoreCoralCommand → prepareScoreFuelCommand, intakeCoralCommand → intakeFuelCommand, etc."
    status: pending
  - id: update-position-tracker
    content: "Rename PositionTracker fields: coralInTraySupplier → fuelInTraySupplier, coralInArmSupplier → fuelInArmSupplier, and update getter methods"
    status: pending
  - id: update-robot-context
    content: "Rename RobotContext field: reefPhotonVision → hubPhotonVision and update constructor"
    status: pending
  - id: update-binding-configurator
    content: Update BindingConfigurator.java to use new command names (centerToHubCommand, prepareScoreFuelCommand, etc.)
    status: pending
  - id: update-constants
    content: "Rename CommandConstants: PLACE_CORAL_FORWARD_SPEED → PLACE_FUEL_FORWARD_SPEED"
    status: pending
  - id: update-scoring-levels
    content: Review and update ScoreLevel enum mapping for 2026 TOWER levels (LEVEL 1, 2, 3) or HUB positions
    status: pending
  - id: update-test-files
    content: "Update all test files: rename CenterToReefCommandTest, update method names in RobotCommandsTest, update field names in PositionTrackerTest and RobotContextTest"
    status: pending
  - id: update-path-versions
    content: Update PathPlanner path file versions from 2025.0 to 2026.0 in all .path and .auto files
    status: pending
  - id: update-documentation
    content: Update all JavaDoc comments and inline documentation to reference FUEL, HUB, TOWER instead of Coral, Reef, etc.
    status: pending
---

# Update Codebase for 2026 REBUILT Game

## ⚠️ Status: This is a TODO/Plan Document

**This document describes planned changes, not the current state of the codebase.**

As of the last review, the codebase still uses 2025 terminology (Coral, Reef) and has not yet been fully migrated to 2026 REBUILT game elements (FUEL, HUB). Many subsystems (Elevator, Arm, SideToSide) have been removed, but the game element renaming has not been completed.

## Overview

Transform the codebase from 2025 Reefscape to 2026 REBUILT game. This includes updating field layouts, AprilTag configurations, renaming game elements (Coral→FUEL, Reef→HUB), updating command names, and adjusting scoring logic.

## Key Game Element Mappings

- **Coral** → **FUEL** (scoring element: 5.91" foam balls)
- **Reef** → **HUB** (scoring target: 47"×47" structure with hexagonal opening)
- **Algae** → **FUEL** (consolidate into single FUEL intake)
- **Processor** → Determine if this maps to **OUTPOST** or **DEPOT** (both are FUEL sources)
- **ScoreLevel L1-L4** → Map to **TOWER levels** (LEVEL 1, LEVEL 2, LEVEL 3) or HUB scoring positions
- **AprilTags** → Update from 2025 IDs (1-16) to 2026 IDs (1-32) with new locations

## Implementation Tasks

### 1. Update Field Layout and AprilTag Configuration

**Files to modify:**

- `src/main/java/frc/robot/Subsystems/Cameras/PhotonVision.java`
  - Change `AprilTagFields.k2025ReefscapeAndyMark` → `AprilTagFields.k2026RebuiltAndyMark` (or equivalent)

- `src/main/java/frc/robot/Commands/CenterToReefCommand.java`
  - Update field layout reference
  - Update AprilTag ID ranges: Currently uses tags 6-11 and 17-22 for "reef" positioning
  - For 2026: HUB AprilTags are IDs 2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27
  - Update tag loop ranges to match 2026 HUB tag IDs

- `src/main/java/frc/robot/PathPlanner/AprilTags/BlueAprilTags.java`
- `src/main/java/frc/robot/PathPlanner/AprilTags/RedAprilTags.java`
- `src/main/java/frc/robot/PathPlanner/AprilTags/AprilTags.java`
  - Update AprilTag ID constants to match 2026 field layout:
    - HUB tags: 2, 3, 4, 5, 8, 9, 10, 11 (blue), 18, 19, 20, 21, 24, 25, 26, 27 (red)
    - TOWER WALL tags: 15, 16 (blue), 31, 32 (red)
    - OUTPOST tags: 13, 14 (blue), 29, 30 (red)
    - TRENCH tags: 1, 6, 7, 12 (blue), 17, 22, 23, 28 (red)
  - Update method names: `OurSpeakerRight/Left` → `OurHubRight/Left`, `OurAmp` → `OurOutpost`, `OurStage1/2/3` → `OurTowerLow/Mid/High`

### 2. Rename Game Element References

**Rename commands and methods:**

- `CenterToReefCommand.java` → `CenterToHubCommand.java`
  - Rename class, update comments, update AprilTag target logic
  - Update references in `BindingConfigurator.java` and `AutoCommandA.java`

- `CenterToProcesserCommand.java` → `CenterToOutpostCommand.java` or `CenterToDepotCommand.java`
  - Determine which element "processor" maps to (OUTPOST for human player feed, DEPOT for field staging)

**Update RobotCommands.java:**

- `prepareScoreCoralCommand` → `prepareScoreFuelCommand`
- `scoreCoralCommand` → `scoreFuelCommand`
- `preIntakeCoralCommand` → `preIntakeFuelCommand`
- `intakeCoralCommand` → `intakeFuelCommand`
- `postIntakeCoralCommand` → `postIntakeFuelCommand`
- `prepareScoreCoralRetryCommand` → `prepareScoreFuelRetryCommand`
- Update all method documentation and parameter names

**Update PositionTracker.java:**

- `coralInTraySupplier` → `fuelInTraySupplier`
- `coralInArmSupplier` → `fuelInArmSupplier`
- `getCoralInTray()` → `getFuelInTray()`
- `getCoralInArm()` → `getFuelInArm()`
- Update comments and documentation

**Update RobotContext.java:**

- `reefPhotonVision` → `hubPhotonVision`
- Update constructor parameter and field name
- Update all references in test files

**Update BindingConfigurator.java:**

- `centerToReefCommand` → `centerToHubCommand`
- `createPlaceCoralForwardCommand` → `createPlaceFuelForwardCommand`
- `prepareScoreCoralAndCenterToReefCommand` → `prepareScoreFuelAndCenterToHubCommand`
- Update all method calls to use new command names

### 3. Update Constants

**CommandConstants.java:**

- `PLACE_CORAL_FORWARD_SPEED` → `PLACE_FUEL_FORWARD_SPEED`

**Subsystem references:**

- Consider renaming `AlgaeIntake.java` → `FuelIntake.java` (or keep name if it's just internal)
- Update `ALGAE_INTAKE` CAN ID constant name if renamed
- Update all references to algae intake throughout codebase

### 4. Update Scoring Logic

**ScoreLevel.java:**

- Determine mapping: L1-L4 may map to:
  - TOWER levels: LEVEL 1 (27" from floor), LEVEL 2 (45" from floor), LEVEL 3 (63" from floor)
  - Or HUB scoring positions (if robot scores from different heights/angles)
- Update enum values or add comments explaining 2026 mapping

**ScoreSide.java:**

- May still apply for HUB scoring (Left/Right/Center positions)
- Or may need updating for TOWER climbing (different approach angles)

**RobotCommands.java:**

- Update `prepareScoreFuelCommand` to map ScoreLevel to appropriate TOWER levels or HUB positions
- Review elevator/arm positions for 2026 field geometry

### 5. Update PathPlanner Configuration

**Path files:**

- Update version in all `.path` files: `"version": "2025.0"` → `"version": "2026.0"`
- Files: All files in `src/main/deploy/pathplanner/paths/` and `src/main/deploy/pathplanner/autos/`
- Note: Path waypoints will need to be recreated for 2026 field layout, but version update is required

**settings.json:**

- Review and update field dimensions if hardcoded

### 6. Update Test Files

**Test files to update:**

- `RobotContextTest.java`: Update `reefPhotonVision` → `hubPhotonVision`
- `RobotCommandsTest.java`: Update all `Coral` → `Fuel` method name references
- `PositionTrackerTest.java`: Update `coral` → `fuel` variable names and method calls
- `CenterToReefCommandTest.java`: Rename to `CenterToHubCommandTest.java` and update references
- `CommandConstantsTest.java`: Update constant name assertions
- `TestUtils.java`: Update `coral` → `fuel` supplier names and comments

### 7. Update Documentation

**Comments and JavaDoc:**

- Update all class/method documentation to reference FUEL, HUB, TOWER instead of Coral, Reef, etc.
- Update field layout descriptions
- Update AprilTag location descriptions

## Implementation Order

1. **Phase 1: Field Layout & AprilTags** (Foundation)
   - Update PhotonVision and CenterToReefCommand field layouts
   - Update AprilTag ID constants
   - Verify WPILib provides k2026RebuiltAndyMark (or equivalent)

2. **Phase 2: Rename Core Elements** (Breaking changes)
   - Rename commands (CenterToReef → CenterToHub, etc.)
   - Update RobotCommands method names
   - Update PositionTracker field names

3. **Phase 3: Update References** (Cascading changes)
   - Update all references in BindingConfigurator, AutoCommandA, etc.
   - Update RobotContext field names
   - Update constants

4. **Phase 4: Update Tests** (Validation)
   - Rename and update all test files
   - Fix test assertions and method calls

5. **Phase 5: PathPlanner & Documentation** (Polish)
   - Update path file versions
   - Update all JavaDoc and comments

## Notes

- **AprilTag Field Layout**: Verify the exact constant name WPILib uses for 2026 (may be `k2026RebuiltAndyMark` or similar)
- **Scoring Levels**: Confirm with team whether ScoreLevel maps to TOWER levels or HUB positions
- **Processor Mapping**: Determine if "processor" maps to OUTPOST (human player feed) or DEPOT (field staging)
- **Path Waypoints**: Path files will need waypoints recreated for 2026 field, but version update is immediate requirement
- **Algae vs Fuel**: If AlgaeIntake and Coral handling are the same mechanism, consider consolidating naming

## Files Summary

**Core files to modify:**

- PhotonVision.java
- CenterToReefCommand.java (rename to CenterToHubCommand.java)
- CenterToProcesserCommand.java (rename)
- RobotCommands.java
- PositionTracker.java
- RobotContext.java
- BindingConfigurator.java
- CommandConstants.java
- AprilTags classes (3 files)
- All test files (6+ files)
- All path files (15+ files)

**Estimated impact:** ~30-40 files modified, 2 files renamed
