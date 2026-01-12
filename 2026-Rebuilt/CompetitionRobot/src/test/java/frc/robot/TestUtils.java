package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Commands.RobotContext;
import frc.robot.State.StateMachine;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.SideToSide;
import frc.robot.Subsystems.Cameras.PhotonVision;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

import static org.mockito.Mockito.mock;

/**
 * Utility class for creating test fixtures and mock objects.
 * Provides helper methods to create test instances of robot components.
 */
public class TestUtils {
    private TestUtils() {
        // Utility class - prevent instantiation
    }

    /**
     * Creates a mock PositionTracker with default suppliers that return 0.0 or false.
     * 
     * @return A PositionTracker instance suitable for testing
     */
    public static PositionTracker createMockPositionTracker() {
        return new PositionTracker(
                () -> 0.0,  // elevator position
                () -> 0.0,  // arm angle
                () -> 0.0,  // side-to-side position
                () -> 0.0,  // climber position
                () -> false, // coral in tray (inverted logic)
                () -> false, // coral in arm (inverted logic)
                () -> 0.0   // algae intake speed
        );
    }

    /**
     * Creates a mock PositionTracker with custom suppliers.
     * 
     * @param elevatorPosition Supplier for elevator position
     * @param armAngle Supplier for arm angle
     * @param sideToSidePosition Supplier for side-to-side position
     * @param climberPosition Supplier for climber position
     * @param coralInTray Supplier for coral in tray sensor
     * @param coralInArm Supplier for coral in arm sensor
     * @param algaeIntakeSpeed Supplier for algae intake speed
     * @return A PositionTracker instance with custom suppliers
     */
    public static PositionTracker createMockPositionTracker(
            Supplier<Double> elevatorPosition,
            Supplier<Double> armAngle,
            Supplier<Double> sideToSidePosition,
            Supplier<Double> climberPosition,
            Supplier<Boolean> coralInTray,
            Supplier<Boolean> coralInArm,
            Supplier<Double> algaeIntakeSpeed) {
        return new PositionTracker(
                elevatorPosition,
                armAngle,
                sideToSidePosition,
                climberPosition,
                coralInTray,
                coralInArm,
                algaeIntakeSpeed
        );
    }

    /**
     * Creates a mock RobotContext with mocked subsystems.
     * Uses Mockito to create mocks for subsystems that are difficult to instantiate.
     * Creates a real StateMachine instance since it has a no-arg constructor.
     * 
     * @return A RobotContext instance suitable for testing
     */
    public static RobotContext createMockRobotContext() {
        // Create a real StateMachine since it has a no-arg constructor
        StateMachine stateMachine = new StateMachine();
        
        // Use Mockito to create mocks for subsystems that require hardware dependencies
        return new RobotContext(
                stateMachine,
                createMockPositionTracker(),
                mock(CommandSwerveDrivetrain.class), // drivetrain
                mock(Elevator.class),               // elevator
                mock(Arm.class),                     // arm
                mock(SideToSide.class),             // sideToSide
                mock(Lights.class),                 // lights
                mock(PhotonVision.class)             // reefPhotonVision
        );
    }

    /**
     * Creates a default Pose3d supplier for testing.
     * 
     * @return A supplier that returns a Pose3d at the origin
     */
    public static Supplier<Pose3d> createDefaultPose3dSupplier() {
        return () -> new Pose3d();
    }
}
