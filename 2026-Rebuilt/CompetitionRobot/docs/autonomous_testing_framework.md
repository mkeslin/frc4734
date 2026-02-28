# Autonomous Testing Framework Documentation

## Overview

The Autonomous Testing Framework provides a comprehensive, driver-station-friendly way to test autonomous commands at three levels:

1. **Atoms**: Individual atomic commands (e.g., `CmdFollowPath`, `CmdSnapToHeading`)
2. **Molecules**: Short 2-3 command sequences (e.g., Seed->Path->Aim)
3. **Full Autos**: Complete autonomous routines (e.g., ClimberAuto, ShooterAuto)

The framework includes automatic telemetry logging, deterministic timeout detection, and comprehensive result tracking to help diagnose why tests fail.

For a **step-by-step description of each autonomous routine** (ClimberAuto, ShooterAuto), selection, and key constants, see [Autonomous Routines](autonomous_routines.md).

## Architecture

### Core Components

- **AutoTestHarness**: Main test harness that manages test registration and execution
- **LoggedCommand**: Wraps commands with telemetry logging and end-reason detection
- **TelemetrySnapshot**: Captures subsystem state at test start and end
- **TimeoutCommand**: Provides deterministic timeout detection via `raceWith()` pattern
- **CommandRunResult**: Records test results with metrics and end reason

### Key Features

- **Deterministic Timeout Detection**: Uses `raceWith()` pattern to reliably detect timeouts
- **Comprehensive Telemetry**: Captures pose, vision, shooter, intake, and climber state
- **Safety**: Prevents re-trigger spam, includes e-stop capability, all tests have timeouts
- **Driver-Station Friendly**: Shuffleboard tab with choosers, toggles, and live telemetry
- **Extensible**: Easy to add new tests via registration methods

## Testing Ladder

Follow this progression when testing autonomous routines:

### 1. Atoms (Individual Commands)

Start by testing individual atomic commands in isolation. This verifies basic functionality before composition.

**Example Atoms:**
- `SeedOdometry`: Verify odometry seeding works
- `FollowPath`: Test path following with a simple path
- `SnapToHeading`: Verify heading control
- `ShooterSpinUp`: Test shooter spin-up
- `AcquireHubAim`: Test vision-based aiming

**When to Use:**
- When debugging a specific command
- When verifying a new command works correctly
- When tuning command parameters (timeouts, tolerances)

### 2. Molecules (2-3 Command Sequences)

Once atoms work individually, test short sequences to verify composition.

**Example Molecules:**
- `Seed->Path->TagSnap->Aim`: Tests odometry seeding, path following, vision correction, and aiming
- `Path+SpinUp->Shoot`: Tests parallel path following and shooter spin-up, then shooting
- `Path->Align->Climb->Hold`: Tests complete climb sequence

**When to Use:**
- When verifying command composition works
- When testing common command sequences
- When debugging interaction between commands

### 3. Full Autos (Complete Routines)

Finally, test complete autonomous routines end-to-end. See [Autonomous Routines](autonomous_routines.md) for a full step-by-step description of each.

**Example Autos:**
- `ClimberAuto`: Complete climber routine with shooting and climbing
- `ShooterAuto`: Complete shooter routine with multiple shots and intakes

**When to Use:**
- When verifying complete autonomous routines
- When testing match-ready autos
- When validating full robot behavior

## Running Tests

### Setup

1. **Enable Subsystems**: Ensure all required subsystems are enabled (currently some are commented out for drivetrain-only testing)
2. **Create AutoConfigurator**: The test harness is initialized in `AutoConfigurator.configureAuto()`
3. **Open Shuffleboard**: Navigate to the "AutoTest" tab

### Test Execution

1. **Select Start Pose**: Choose LEFT, CENTER, or RIGHT from the Start Pose chooser
   - This determines which start position and paths are used
   - Path naming convention: `L_StartToShot`, `C_StartToShot`, `R_StartToShot`

2. **Select Test**: Choose a test from the Test Selection chooser
   - Atoms are prefixed with "TEST: Atom - "
   - Molecules are prefixed with "TEST: Molecule - "
   - Full autos are prefixed with "AUTO: "

3. **Run Test**: Toggle the "Run Test" button to true
   - Test will start immediately if no other test is running
   - Re-triggering is prevented while a test is active

4. **Monitor Progress**: Watch live telemetry values:
   - `poseX`, `poseY`: Robot position
   - `headingErrorDeg`: Heading error from target
   - `tagCount`, `tagAmbiguity`: Vision quality
   - `shooterRpmActual`, `shooterAtSpeed`: Shooter state
   - `ballCount`: Game piece count
   - `climbQualified`: Climber position status

5. **Check Results**: After test completes, check "Last Result" section:
   - **Name**: Test that ran
   - **Reason**: Why it ended (SUCCESS, TIMEOUT, INTERRUPTED, CONDITION_FALSE, ERROR)
   - **Duration**: How long it took

6. **Emergency Stop**: If needed, toggle "Stop Test" to immediately stop and run `CmdStopAll`

### Test States

- **Not Running**: No test active, "Run Test" toggle can be used
- **Running**: Test is executing, "Run Test" toggle is ignored
- **Finished**: Test completed, results available in "Last Result"

## Interpreting Results

### End Reasons

#### SUCCESS
- Command completed normally
- All conditions met (tolerances, timeouts, etc.)
- **Action**: Test passed, proceed to next level or tune parameters

#### TIMEOUT
- Command exceeded its timeout duration
- **Possible Causes**:
  - Command is too slow
  - Timeout value too short
  - Robot stuck or blocked
- **Troubleshooting**:
  - Check if robot is physically blocked
  - Increase timeout in command or AutoConstants
  - Verify path following is working (check pose telemetry)
  - Check for mechanical issues

#### INTERRUPTED
- Command was canceled externally
- **Possible Causes**:
  - "Stop Test" button pressed
  - Command canceled by another command
  - Robot disabled
- **Action**: Normal if intentional, otherwise check for unexpected interruptions

#### CONDITION_FALSE
- Command ended because a condition was not met
- **Possible Causes**:
  - Vision quality too poor (tagCount < minTargets, ambiguity too high)
  - Tolerance not met (pose, heading, RPM)
  - Sensor reading incorrect
- **Troubleshooting**:
  - Check vision metrics: `tagCount`, `tagAmbiguity`, `tagDistance`
  - Verify tolerances are reasonable
  - Check sensor readings (ball count, climber position)
  - Review command logic for condition checks

#### ERROR
- Unexpected error occurred
- **Possible Causes**:
  - Exception thrown
  - Subsystem not initialized
  - Null pointer exception
- **Troubleshooting**:
  - Check robot logs for exceptions
  - Verify all subsystems are initialized
  - Check for null references in command

### Telemetry Analysis

Compare `metricsStart` vs `metricsEnd` to understand what changed:

#### Pose Changes
- **Expected**: Robot should move to target pose
- **Unexpected**: Robot didn't move, or moved to wrong location
  - Check path file exists and is valid
  - Verify drivetrain is working
  - Check for odometry drift

#### Heading Error
- **Expected**: `headingErrorDeg` should decrease to near zero
- **Unexpected**: Heading error remains large
  - Check heading control is working
  - Verify target heading is correct
  - Check for mechanical binding

#### Vision Quality
- **Good**: `tagCount >= 2`, `tagAmbiguity < 0.2`, `tagDistance < 5.0m`
- **Poor**: Low tag count, high ambiguity, or far distance
  - Check camera is working
  - Verify AprilTags are visible
  - Check camera position and angle
  - Verify field layout is correct

#### Shooter State
- **Expected**: `shooterRpmActual` should reach target, `shooterAtSpeed = 1.0`
- **Unexpected**: RPM not reaching target
  - Check shooter motor is working
  - Verify target RPM is achievable
  - Check for mechanical issues
  - Verify battery voltage is sufficient

#### Ball Count
- **Expected**: Should increase when intaking
- **Unexpected**: Count not changing
  - Check intake is running
  - Verify sensor is working
  - Check for mechanical blockage

#### Climb Qualified
- **Expected**: Should be `1.0` when climber is at CLIMB position
- **Unexpected**: Remains `0.0`
  - Check climber position
  - Verify climber is moving
  - Check for mechanical issues

## Troubleshooting Guide

### Test Won't Start

**Symptoms**: "Run Test" toggle does nothing, test doesn't start

**Possible Causes**:
1. Another test is already running
2. No test selected in chooser
3. Test harness not initialized

**Solutions**:
- Check "AutoTest/Status" in SmartDashboard for current state
- Ensure a test is selected in the chooser
- Verify `AutoConfigurator.configureAuto()` was called
- Check that test harness was created successfully

### Test Times Out Immediately

**Symptoms**: Test ends with TIMEOUT reason almost instantly

**Possible Causes**:
1. Timeout value too short
2. Command not starting properly
3. Command finishing immediately (empty command)

**Solutions**:
- Check timeout value in command or AutoConstants
- Verify command is actually executing (check logs)
- Ensure command has proper initialization
- Check if command is returning empty/None command

### Vision Commands Fail

**Symptoms**: Commands using vision end with CONDITION_FALSE or TIMEOUT

**Possible Causes**:
1. No AprilTags visible
2. Camera not working
3. Vision thresholds too strict
4. Field layout incorrect

**Solutions**:
- Check `tagCount` in live telemetry (should be >= 2)
- Verify camera is connected and working
- Check `tagAmbiguity` (should be < 0.2)
- Verify AprilTag IDs match field layout
- Check camera position and angle
- Adjust `DEFAULT_MAX_AMBIGUITY`, `DEFAULT_MAX_TAG_DISTANCE`, `DEFAULT_MIN_TARGETS` in AutoConstants

### Path Following Issues

**Symptoms**: Robot doesn't follow path, or path command times out

**Possible Causes**:
1. Path file missing or invalid
2. Path name incorrect
3. Drivetrain not working
4. Odometry not seeded

**Solutions**:
- Verify path file exists in deploy directory
- Check path name matches exactly (case-sensitive)
- Verify drivetrain is working (check pose telemetry)
- Ensure odometry is seeded before path following
- Check path file format is valid PathPlanner format

### Shooter Not Reaching Speed

**Symptoms**: `shooterAtSpeed` remains 0.0, test times out

**Possible Causes**:
1. Target RPM too high
2. Shooter motor not working
3. Battery voltage too low
4. Mechanical binding

**Solutions**:
- Check `shooterRpmActual` in telemetry
- Verify target RPM is achievable
- Check shooter motor is running
- Verify battery voltage is sufficient
- Check for mechanical issues
- Adjust RPM tolerance if needed

### Robot Doesn't Move

**Symptoms**: Pose doesn't change during test

**Possible Causes**:
1. Drivetrain not enabled
2. Robot disabled
3. Command not executing
4. Mechanical issue

**Solutions**:
- Verify robot is enabled
- Check drivetrain is working
- Verify command is actually scheduled
- Check for mechanical binding
- Verify odometry is updating

### Adding New Tests

To add a new atom test:

```java
m_testHarness.registerAtom("MyNewCommand", () -> 
    new MyNewCommand(required, parameters));
```

To add a new molecule test:

```java
m_testHarness.registerMolecule("MyNewMolecule", () -> 
    Commands.sequence(
        new Command1(),
        new Command2(),
        new Command3()
    ));
```

To add a new full auto:

```java
Command myAuto = AutoRoutines.buildMyAuto(...);
m_autoManager.addRoutine(new AutoRoutine("MyAuto", myAuto));
```

## Best Practices

1. **Start Small**: Always test atoms before molecules, molecules before full autos
2. **Check Telemetry**: Always review telemetry when a test fails
3. **Use Timeouts**: All tests should have reasonable timeouts
4. **Test in Isolation**: Test one thing at a time when debugging
5. **Document Failures**: Note what failed and why for future reference
6. **Iterate**: Fix issues at the atom level before moving to molecules
7. **Verify Assumptions**: Check that subsystems are working before testing commands
8. **Use Live Telemetry**: Monitor live values during test execution

## NetworkTables Entries

The test harness publishes the following NetworkTables entries:

- `/AutoTest/SelectedTest`: Currently selected test name
- `/AutoTest/Run`: Boolean toggle to start test
- `/AutoTest/Stop`: Boolean toggle to stop test
- `/AutoTest/LastResult/Name`: Last test name
- `/AutoTest/LastResult/Reason`: Last test end reason
- `/AutoTest/LastResult/Duration`: Last test duration
- `/AutoTest/Live/poseX`, `/AutoTest/Live/poseY`: Current robot position
- `/AutoTest/Live/headingErrorDeg`: Current heading error
- `/AutoTest/Live/tagCount`, `/AutoTest/Live/tagAmbiguity`: Vision metrics
- `/AutoTest/Live/shooterRpmActual`, `/AutoTest/Live/shooterAtSpeed`: Shooter state
- `/AutoTest/Live/ballCount`: Game piece count
- `/AutoTest/Live/climbQualified`: Climber status

## Example Workflow

1. **Test Atom**: `FollowPath`
   - Select "L_StartToShot" path
   - Run test
   - Verify robot follows path correctly
   - Check pose telemetry shows movement

2. **Test Molecule**: `Seed->Path->TagSnap->Aim`
   - Run molecule test
   - Verify each step completes
   - Check vision quality metrics
   - Verify final heading is correct

3. **Test Full Auto**: `ClimberAuto`
   - Run complete auto
   - Monitor all subsystems
   - Verify complete sequence works
   - Check final state matches expected

## Additional Resources

- **AutoConstants**: Default timeouts and tolerances
- **Atomic Commands**: Individual command documentation
- **AutoRoutines**: Full auto builder methods
- **PathPlanner**: Path file format and usage

## Support

For issues or questions:
1. Check this documentation
2. Review telemetry and logs
3. Verify subsystems are working
4. Check command implementations
5. Consult team members
