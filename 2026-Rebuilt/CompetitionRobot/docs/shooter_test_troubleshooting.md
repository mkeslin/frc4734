# Shooter / AutoTest Troubleshooting

If **ShooterOn** (or other shooter/feeder test atoms) do nothing when you run them from the AutoTest tab, check the following.

## 1. Run the test while the robot is **enabled**

The AutoTest "Run" toggle is polled in both **Disabled** and **Enabled** (Teleop). Make sure:

- You are in **Teleop** (or Disabled if you prefer; the command will run when the scheduler runs).
- Open the **AutoTest** tab in Shuffleboard.
- Select **TEST: Atom - ShooterOn** (or another test) from the dropdown.
- Set **Run** to **true** (toggle or button). The command is scheduled and should run.

If you only ever toggle Run while in Disabled, the command runs while disabled; when you enable, it may have already finished or been cancelled depending on the command.

## 2. Check "Robot Initialized" on the Driver tab

The shooter (and feeder) **do not run** until `RobotState` is initialized. The Driver tab now shows shooter status with **init:yes** or **init:no**.

- **init:no** → Mechanisms are blocked. Initialization is scheduled once at robot boot in `RobotContainer`; if you see init:no, check that the robot has completed startup and that no code is calling `disableInitializationCommand()`.
- **init:yes** → Mechanisms are allowed to run.

## 3. Shooter speed is visible

- **ShooterOn** in the test harness uses **30 rotations per second** (~1800 RPM) so the wheels should be clearly spinning. If you previously used `ShooterSpeed.FORWARD` (0.5 RPS), that is too low to see.
- **Shooter Status** on the Driver tab shows READY or SPINNING and the current RPM when the shooter subsystem is present.

## 4. CAN and wiring

- Shooter motors: CAN IDs **29** (left/leader), **30** (right), **32** (center). Confirm device IDs in Phoenix Tuner or via CAN bus check.
- Feeder: CAN ID **28**. Confirm wiring and that no other device uses the same ID.

## 5. Stop a running test

Use the AutoTest **Stop** toggle to cancel the current test and run **StopAll** (stops drivetrain, shooter, feeder).

## 6. Controller as alternative

You can also drive the shooter from the **mechanism controller** (second Xbox controller):

- **Right bumper (hold)** = Shooter forward
- **Left bumper (hold)** = Shooter reverse  
- **Back** = Shooter off

If the controller works but the AutoTest tab does not, the issue is with how/when the test harness runs the selected command (e.g. Run not toggled, or wrong test selected).
