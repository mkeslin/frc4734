// package frc.robot.Subsystems;

// import static frc.robot.Constants.Constants.*;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class River extends SubsystemBase {
//     private TalonFX m_river;

//     public River() {
//         m_river = new TalonFX(RIVER_ID);

//         m_river.setInverted(false);
//         m_river.setNeutralMode(NeutralModeValue.Brake);
//         var configs = new TalonFXConfiguration();
//         configs.CurrentLimits = new CurrentLimitsConfigs();
//         configs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
//         m_river.getConfigurator().apply(configs);
//     }

//     public Command commandStartIn() {
//         return Commands.runOnce(() -> this.startIn(-.55));
//     }

//     public Command commandStartOut() {
//         return Commands.runOnce(() -> this.startOut());
//     }

//     public Command commandStopRiver() {
//         return Commands.runOnce(() -> this.stopRiver());
//     }

//     public boolean isOn() {
//         return getSpeed() > 0;
//     }

//     public double getSpeed() {
//         return m_river.get();
//     }

//     public void startIn(double s) {
//         m_river.set(s);
//     }

//     public void startOut() {
//         m_river.set(-.25);
//     }

//     public void stopRiver() {
//         m_river.set(0);
//     }


//     // @Override
//     // public void periodic() {}

//     // @Override
//     // public void initDefaultCommand() {}

// }
