# Pit Check Integration Guide

## Quick Start

Add the following to your `RobotContainer.java`:

### 1. Add Fields

```java
private PitCheckRunner pitCheckRunner;
private PitCheckShuffleboard pitCheckShuffleboard;
```

### 2. Create Concrete Step Implementations

Create concrete implementations of the abstract step classes. Example for intake:

```java
private static class IntakePulseTestStepImpl extends IntakePulseTestStep {
    private final DeployableIntake intake;
    
    public IntakePulseTestStepImpl(DeployableIntake intake) {
        this.intake = intake;
    }
    
    @Override
    protected void setIntakePower(double power) {
        intake.setPower(power); // Adapt to your method
    }
    
    @Override
    protected Supplier<Double> getIntakePositionSupplier() {
        return intake::getDeployPosition; // Adapt to your method
    }
    
    @Override
    protected Supplier<Double> getIntakeVelocitySupplier() {
        return intake::getIntakeSpeed; // Adapt to your method
    }
    
    @Override
    protected Supplier<Double> getIntakeCurrentSupplier() {
        return null; // If no current sensor
    }
    
    @Override
    protected Supplier<Double> getIntakeVoltageSupplier() {
        return () -> RobotController.getBatteryVoltage();
    }
}
```

### 3. Initialize in Constructor

```java
public RobotContainer() {
    // ... existing code ...
    
    // Create stop all runnable
    Runnable stopAll = () -> {
        m_drivetrain.stop(); // Adapt to your methods
        // ... stop other subsystems ...
    };
    
    // Create steps list
    List<PitCheckStep> steps = new ArrayList<>();
    steps.add(new GateCheckStep());
    steps.add(new MyDrivePulseTestStep(m_drivetrain));
    steps.add(new IntakePulseTestStepImpl(m_deployableIntake));
    // ... add other steps ...
    
    // Create shuffleboard
    pitCheckShuffleboard = new PitCheckShuffleboard();
    
    // Create runner
    pitCheckRunner = new PitCheckRunner(steps, pitCheckShuffleboard, stopAll);
    
    // Configure buttons
    configurePitCheckButtons();
}
```

### 4. Configure Buttons

```java
private void configurePitCheckButtons() {
    // Run pit check (only when disabled/test, not FMS)
    new Trigger(() -> {
        return !DriverStation.isFMSAttached() &&
               (DriverStation.isDisabled() || DriverStation.isTest()) &&
               pitCheckShuffleboard.getRunPitCheck() &&
               !pitCheckRunner.isRunning();
    }).onTrue(pitCheckRunner);
    
    // Abort
    new Trigger(() -> pitCheckShuffleboard.getAbortPitCheck())
        .onTrue(Commands.runOnce(() -> {
            pitCheckRunner.cancel();
            pitCheckShuffleboard.resetControls();
        }));
    
    // Run selected step
    new Trigger(() -> pitCheckShuffleboard.getRunSelectedStep())
        .onTrue(Commands.runOnce(() -> {
            pitCheckRunner.runSelectedStep();
            pitCheckShuffleboard.resetControls();
        }));
}
```

## Safety Gates

The framework automatically enforces:
- ✅ No execution if FMS attached
- ✅ Drivetrain steps require "RobotOnBlocks" toggle
- ✅ Climber motion requires "ClimberMotionAllowed" toggle
- ✅ Abort immediately stops all subsystems

## Shuffleboard Tab

The "PitCheck" tab provides:
- **Controls**: Run, Abort, RobotOnBlocks, ClimberMotionAllowed
- **Status**: Current state, step name, results
- **Step Chooser**: Select individual step to rerun

## Expected Runtime

- Full check: 30-45 seconds
- Individual step: 1-5 seconds
