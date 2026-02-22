package frc.robot.Constants;

import frc.robot.SwerveDrivetrain.SwerveDrivetrainBindings.InputProfile;

/**
 * Constants for controller binding behavior and profile selection.
 * Change these to lock the robot to a single profile or allow runtime switching.
 */
public final class ControllerBindingConstants {
    private ControllerBindingConstants() {}

    /**
     * When true, Back + Start on the drive controller cycles TELEOP → SYSID → MECHANISM → TELEOP.
     * When false, the drive profile stays at {@link #DEFAULT_DRIVE_PROFILE} for the session.
     */
    public static final boolean ENABLE_PROFILE_SWITCHING = true;

    /**
     * Initial drive profile at boot. If {@link #ENABLE_PROFILE_SWITCHING} is false,
     * this is the only profile used for the session.
     */
    public static final InputProfile DEFAULT_DRIVE_PROFILE = InputProfile.TELEOP;
}
