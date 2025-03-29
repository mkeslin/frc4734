package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PositionTracker;
import frc.robot.Subsystems.Cameras.VisionCamera;
import frc.robot.SwerveDrivetrain.CommandSwerveDrivetrain;

public class CenterToStationCommand extends Command {
    public PositionTracker m_positionTracker;
    public VisionCamera m_camera;
    public CommandSwerveDrivetrain m_drivetrain;
    public CommandXboxController m_driveController;

    private boolean driverInterrupted;

    private final PIDController xController = new PIDController(.3, 0, 0);
    private final PIDController yController = new PIDController(.1, 0, 0);
    private final PIDController omegaController = new PIDController(0.03, 0, 0);

    private double AREA_GOAL = 4.2;
    private double AREA_ERROR = 2;
    private double CAMERA_X_OFFSET_ERROR = 1;
    private double ANGLE_ERROR = 3;

    public Timer t = new Timer();
    public Timer t_tray = new Timer();

    public CenterToStationCommand(PositionTracker positionTracker, VisionCamera camera,
            CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
        m_positionTracker = positionTracker;
        m_camera = camera;
        m_drivetrain = drivetrain;
        m_driveController = driveController;

        xController.setTolerance(AREA_ERROR);
        yController.setTolerance(CAMERA_X_OFFSET_ERROR);
        omegaController.setTolerance(ANGLE_ERROR);

        addRequirements(m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        t.start();

        xController.setSetpoint(AREA_GOAL);
        yController.setSetpoint(0);
        omegaController.setSetpoint(0);
        driverInterrupted = false;
    }

    @Override
    public void execute() {
        var xSpeed = -xController.calculate(m_camera.getArea());
        var ySpeed = -yController.calculate(m_camera.getX());
        var omegaSpeed = omegaController.calculate(Units.radiansToDegrees(m_camera.getYaw()));
        if (!m_camera.hasTargets()) {
            ySpeed = 0;
            xSpeed = 0;
            omegaSpeed = 0;
        }
        // getSpeeds(xSpeed, ySpeed, omegaSpeed);
        m_drivetrain.setRelativeSpeed(xSpeed, ySpeed, omegaSpeed);
        if (m_driveController != null) {
            m_driveController.povUp().onTrue(Commands.runOnce(() -> {
                driverInterrupted = true;
            }));
        }

        if (m_positionTracker.getCoralInTray() && !t_tray.isRunning()) {
            t_tray.start();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (t.hasElapsed(5)) {
            System.out.println("Center to Station: time elapsed");
            return true;
        }
        if (driverInterrupted) {
            System.out.println("Center to Station: driver interrupted");
            return true;
        }
        if (t_tray.hasElapsed(.75)) {
            System.out.println("Center to Station: coral acquired");
            return true;
        }
        if (!m_camera.hasTargets()) {
            System.out.println("Center to Station: lost tag");
            return true;
        }
        if (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()) {
            System.out.println("Center to Station: centered");
            return true;
        }

        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        t.stop();
        t.reset();

        t_tray.stop();
        t_tray.reset();
    }

    public void getSpeeds(double x, double y, double omega) {
        SmartDashboard.putNumber("xSpeed", x);
        SmartDashboard.putNumber("ySpeed", y);
        SmartDashboard.putNumber("omegaSpeed", omega);
    }
}