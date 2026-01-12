# PathPlanner Configuration Review

## Summary
PathPlanner is **mostly correctly configured** according to 2026 specifications. There is one potential issue that should be verified.

## ✅ Correctly Configured Components

### 1. AutoBuilder Configuration
- **Location**: `CommandSwerveDrivetrain.configureAutoBuilder()`
- **Status**: ✅ Correct
- Configured in constructor, loads `RobotConfig` from GUI settings
- Exception handling in place

### 2. Pose Supplier
- **Code**: `() -> getState().Pose`
- **Status**: ✅ Correct
- Returns current robot pose from Phoenix 6's state

### 3. Reset Pose Method
- **Code**: `this::resetPose`
- **Status**: ✅ Correct
- Phoenix 6's `resetPose(Pose2d)` method exists (renamed from `seedFieldRelative` in 2026)
- Properly used in `AutoManager` to reset odometry before autonomous

### 4. Drive Method with Feedforwards
- **Code**: 
  ```java
  (speeds, feedforwards) -> setControl(
      m_pathApplyRobotSpeeds.withSpeeds(speeds)
          .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
          .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()))
  ```
- **Status**: ✅ Correct
- Properly applies robot-relative speeds and feedforwards
- Uses `SwerveRequest.ApplyRobotSpeeds` which is correct for PathPlanner

### 5. Path Following Controller
- **Code**: `new PPHolonomicDriveController(new PIDConstants(7, 0, 0), new PIDConstants(7, 0, 0))`
- **Status**: ✅ Correct
- Uses `PPHolonomicDriveController` for swerve drive
- PID constants configured (may need tuning based on robot performance)

### 6. Alliance Flipping
- **Code**: `() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red`
- **Status**: ✅ Correct
- Properly detects Red alliance for path mirroring

### 7. Path Loading
- **Code**: `PathPlannerPath.fromPathFile("path-name")`
- **Status**: ✅ Correct
- Paths loaded from deploy directory
- Error handling in place

### 8. Path Following
- **Code**: `AutoBuilder.followPath(path)`
- **Status**: ✅ Correct
- Uses PathPlanner's AutoBuilder correctly
- Wrapped in `followPathCommand()` method

### 9. Odometry Reset
- **Location**: `AutoManager.runSelectedRoutine()`
- **Status**: ✅ Correct
- Resets pose before autonomous starts using `beforeStarting()`
- Uses initial pose from path's `getStartingDifferentialPose()`

## ⚠️ Potential Issue

### Speeds Supplier
- **Current Code**: `() -> getState().Speeds`
- **Status**: ⚠️ **Needs Verification**
- **Requirement**: PathPlanner requires **robot-relative** ChassisSpeeds
- **Analysis**: 
  - Phoenix 6's `SwerveDriveState.Speeds` should be robot-relative (internal state)
  - The drive method uses `ApplyRobotSpeeds`, confirming robot-relative expectation
  - However, this should be verified through testing or Phoenix 6 documentation

**Recommendation**: 
1. Test autonomous path following to verify speeds are interpreted correctly
2. If issues occur, consider explicitly converting to robot-relative:
   ```java
   () -> {
       var state = getState();
       // If Speeds is field-relative, convert:
       return ChassisSpeeds.fromFieldRelativeSpeeds(
           state.Speeds.vxMetersPerSecond,
           state.Speeds.vyMetersPerSecond,
           state.Speeds.omegaRadiansPerSecond,
           state.Pose.getRotation()
       );
   }
   ```
3. Check Phoenix 6 documentation to confirm `SwerveDriveState.Speeds` is robot-relative

## 📝 Additional Notes

### PID Tuning
- Current PID constants: `(7, 0, 0)` for both translation and rotation
- These may need tuning based on robot performance
- Consider using SysId to determine optimal values

### Path Constraints
- Pathfinding constraints in `moveToPose()`:
  - Max angular velocity: 80 deg/s
  - Max angular acceleration: 50 deg/s²
- These seem reasonable but may need adjustment

### Error Handling
- AutoBuilder configuration has try-catch block ✅
- Path loading has error handling ✅
- Failed path loads return empty commands ✅

## ✅ Conclusion

PathPlanner is **correctly configured** according to 2026 specifications. The only item requiring verification is whether `getState().Speeds` returns robot-relative speeds, which should be the case based on Phoenix 6's architecture. If path following works correctly in testing, the configuration is fully correct.

## Testing Recommendations

1. **Test Basic Path Following**: Run a simple path and verify robot follows it accurately
2. **Test Alliance Flipping**: Verify paths mirror correctly on Red alliance
3. **Test Odometry Reset**: Verify robot starts at correct pose after reset
4. **Monitor Speeds**: Log the speeds during path following to verify they're robot-relative
5. **Tune PID Constants**: Adjust PID values if path following is inaccurate or oscillatory
