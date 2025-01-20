package frc.robot.subsystems.NoteHandling;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GeneralRollerConstants.*;
import org.opencv.features2d.MSER;
import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class GeneralRoller extends SubsystemBase {

  public enum GeneralRollerStates {
    StateOff,
    StateForward,
    StateReverse,
  }
  
  public LinearFilter filter = LinearFilter.singlePoleIIR(0.5, 0.2);

  private final CANSparkMax m_spark;
  private double desiredVoltage = 0;
  private GeneralRollerStates currentState = GeneralRollerStates.StateOff;

  public GeneralRoller(int port, boolean setInverted) {
    m_spark = new CANSparkMax(port, MotorType.kBrushless);

   
    m_spark.setInverted(setInverted);
    m_spark.setIdleMode(IdleMode.kBrake);
    m_spark.setSmartCurrentLimit(40); 
    m_spark.enableVoltageCompensation(12);

  }

  @Override
  public void periodic() {
    switch (currentState) {
      case StateOff:
        desiredVoltage = 0;
        break;
      case StateForward:
        desiredVoltage = 6;
        break;
      case StateReverse:
        desiredVoltage = -6;
        break;
      default:
        break;
    }
  }

  public double getCurrent() {
    return m_spark.getOutputCurrent(); 
  }
  
  public void requestState(GeneralRollerStates desiredState) {
    currentState = desiredState;
    m_spark.set(desiredVoltage);
  }

  public GeneralRollerStates getCurrentState() { 
    return currentState;
  }

}