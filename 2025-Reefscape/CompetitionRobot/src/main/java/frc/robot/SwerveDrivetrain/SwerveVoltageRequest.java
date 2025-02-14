package frc.robot.SwerveDrivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SwerveVoltageRequest implements SwerveRequest {
    private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0);
    private final VoltageOut m_voltageOutControl = new VoltageOut(0.0);

    private double m_targetVoltage = 0.0;
    private boolean m_driveType = true;

    public SwerveVoltageRequest(boolean driveType) {
        m_driveType = driveType;
    }

    public SwerveVoltageRequest() {
        m_driveType = true;
    }

    @SuppressWarnings("rawtypes")
    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        for (var module : modulesToApply) 
        {
            // DRIVE TESTS
            if (m_driveType) {
                // Command steer motor to zero
                module.getSteerMotor().setControl(m_motionMagicControl);

                // Command drive motor to voltage
                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));
            }
            // STEER TESTS
            else {
                // Command steer motor to voltage
                module.getSteerMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));

                // Command drive motor to zero
                module.getDriveMotor().setControl(m_motionMagicControl);
            }
        }

        return StatusCode.OK;
    }

    /**
     * 
     * @param targetVoltage Voltage for all modules to target
     * @return 
     */
    public SwerveVoltageRequest withVoltage(double targetVoltage) {
        this.m_targetVoltage = targetVoltage;
        return this;
    }
}
