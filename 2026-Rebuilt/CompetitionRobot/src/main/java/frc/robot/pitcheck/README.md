# Pit Check Testing Framework

A comprehensive pre-match hardware verification system for FRC robots.

## Overview

The Pit Check framework provides a fast (≤45s), safe, and comprehensive way to verify all robot subsystems before each match. It runs a series of automated tests and reports PASS/WARN/FAIL results on Shuffleboard.

## Package Structure

```
frc.robot.pitcheck/
├── PitCheckRunner.java          # Main state machine orchestrator
├── PitCheckStep.java             # Interface for test steps
├── PitCheckResult.java           # Result model (PASS/WARN/FAIL)
├── PitCheckReport.java           # Aggregates all results
├── PitCheckTelemetry.java        # Telemetry sampling utilities
├── PitCheckShuffleboard.java     # Shuffleboard UI manager
├── PitCheckConstants.java        # Configuration constants
├── MotorPulseTestStep.java       # Generic motor pulse test base
└── steps/
    ├── GateCheckStep.java        # Safety gate verification
    ├── DrivePulseTestStep.java   # Drive motor test (abstract)
    ├── SteerPulseTestStep.java   # Steer motor test (abstract)
    ├── IntakePulseTestStep.java  # Intake test (abstract)
    ├── FeederPulseTestStep.java  # Feeder test (abstract)
    ├── ShooterSpinTestStep.java  # Shooter test (abstract)
    ├── VisionHeartbeatTestStep.java # Vision test (abstract)
    └── ClimberSensorTestStep.java   # Climber test (abstract)
```

## Key Features

- **Fast**: Complete check in 30-45 seconds
- **Safe**: Multiple safety gates prevent unsafe operation
- **Comprehensive**: Tests all major subsystems
- **Clear Results**: PASS/WARN/FAIL with detailed messages
- **Flexible**: Easy to add custom test steps
- **Selective**: Can rerun individual steps

## Safety Gates

1. **FMS Check**: Will not run if FMS is attached
2. **Mode Check**: Only runs in Disabled or Test mode
3. **Blocks Required**: Drivetrain tests require "RobotOnBlocks" toggle
4. **Climber Motion**: Requires explicit "ClimberMotionAllowed" toggle
5. **Abort Capability**: Can be aborted at any time

## Default Test Steps

1. **Gate Check**: Verifies safety conditions
2. **Drive Pulse**: Tests drive motors (requires blocks)
3. **Steer Pulse**: Tests steering motors (requires blocks)
4. **Intake Pulse**: Tests intake motor
5. **Feeder Pulse**: Tests feeder motor
6. **Shooter Spin**: Tests shooter at low RPM
7. **Vision Heartbeat**: Verifies camera connectivity
8. **Climber Sensor**: Checks sensors (optional motion)

## Usage

See `docs/pitcheck_integration.md` for integration instructions.

## Result Interpretation

- **PASS**: All checks passed, robot ready
- **WARN**: Marginal values but acceptable (e.g., current slightly low)
- **FAIL**: Critical issue (motor unplugged, encoder not moving, etc.)

## Customization

To add custom test steps:

1. Implement `PitCheckStep` interface or extend `MotorPulseTestStep`
2. Add step to steps list when creating `PitCheckRunner`
3. Step will automatically appear in Shuffleboard chooser

## Constants

All timing, power levels, and thresholds are in `PitCheckConstants.java`. Adjust as needed for your robot.
