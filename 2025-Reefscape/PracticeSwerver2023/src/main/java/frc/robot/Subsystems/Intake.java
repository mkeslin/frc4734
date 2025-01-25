// package frc.robot.Subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticHub;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import static frc.robot.Constants.*;

// public class Intake {

//     private TalonFX wheels1;
//     private TalonFX wheels2;
//     private PneumaticHub hub;
//     private DoubleSolenoid sol;
//     private Compressor compressor;

//     public Intake() {
//         wheels1 = new TalonFX(INTAKELEFTID);
//         wheels2 = new TalonFX(INTAKERIGHTID);
//         hub = new PneumaticHub();
//         sol = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);
//         compressor = new Compressor(1, PneumaticsModuleType.REVPH);
//     }

//     public void open() {
//         sol.set(Value.kReverse);
//         SmartDashboard.putBoolean("Comp", compressor.isEnabled());
//     }

//     public void close() {
//         sol.set(Value.kForward);
//     }

//     public void wheelsIn() {
//         wheels1.set(ControlMode.PercentOutput, -0.25);
//         wheels2.set(ControlMode.PercentOutput, 0.25);
//     }

//     public void wheelsOut() {
//         wheels1.set(ControlMode.PercentOutput, 0.25);
//         wheels2.set(ControlMode.PercentOutput, -0.25);
//     }

//     public void zero() {
//         wheels1.set(ControlMode.PercentOutput, 0);
//         wheels2.set(ControlMode.PercentOutput, 0);
//     }

//     public void enableCompressor() {
//         compressor.enableDigital();
//         hub.enableCompressorAnalog(50, 120);
//     }
// }
