package frc.robot.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PathPlanner.PathPlanner;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LimelightAligner;

/*
 * Use limelight to find note
 * Move robot to correct position
 * Intake note
 */
public class AutoCommand extends Command {

    private final PathPlanner m_pathPlanner;
    private final Intake m_intake;
    private final LimelightAligner m_limelightAligner;

    private AutoState m_state = AutoState.Idling;

    public AutoCommand(PathPlanner pathPlanner, Intake intake, LimelightAligner limelightAligner) {
        m_pathPlanner = pathPlanner;
        m_intake = intake;
        m_limelightAligner = limelightAligner;

        addRequirements(m_pathPlanner, m_intake, m_limelightAligner);
        // addCommands(
        //     // move and acquire
        //     pathPlanner.moveToOurRing1(),
        //     limelightAligner.alignToNote(),
        //     intake.commandStartIn(),
        //     Commands.runOnce(() -> pathPlanner.moveForwardRobot(.5), pathPlanner),
        //     intake.commandStop(),
        //     // shoot
        //     limelightAligner.alignToTag(3)
        //     // Commands.print("acquireNote-333333333333333333333333333333333333333")
        // );
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        SmartDashboard.putString("auto command", "I'm here!!!");
        System.out.println("autoCommand ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

        Commands.sequence(
            // move and acquire
            m_pathPlanner.moveToOurRing1(),
            m_limelightAligner.alignToNote(),
            m_intake.commandStartIn(),
            Commands.runOnce(() -> m_pathPlanner.moveForwardRobot(.5), m_pathPlanner),
            m_intake.commandStopRoller(),
            // shoot
            m_limelightAligner.alignToTag(3)
            // Commands.print("acquireNote-333333333333333333333333333333333333333")
        );
    }

    @Override
    public void execute() {
        // System.out.println("autoCommand - execute $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");

        // switch (m_state) {
        //     case Idling:
        //         m_state = AutoState.Driving;
        //         return;
        // }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {}
}
