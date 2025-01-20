// Import pkgs & classes
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

// GeneralRoller extends SubsystemBase
// This class controls the roller in the robot
public class GeneralRoller extends SubsystemBase {

  public enum GeneralRollerStates {
    StateOff, // Turned off, no voltage
    StateIntake, // Intake
    StateOuttake, // Outtake
  }

  private final CANSparkMax m_spark;
  private double desiredVoltage = 0;
  private GeneralRollerStates currentState = GeneralRollerStates.StateOff;

  // Constructor 
  public GeneralRoller(int port, boolean setInverted) {
    m_spark = new CANSparkMax(port, MotorType.kBrushless);
    m_spark.setInverted(setInverted);
    m_spark.setIdleMode(IdleMode.kBrake);
    m_spark.setSmartCurrentLimit(40);
    m_spark.enableVoltageCompensation(12);
  }

  // Periodic function, checks the current state and sets the voltage based on the state
  @Override
  public void periodic() {

    switch (currentState) {
      case StateOff:
        desiredVoltage = 0;
        break;
      case StateIntake:
        desiredVoltage = 6;
        break;
      case StateOuttake:
        desiredVoltage = -6;
        break;
      default:
        break;
    }

    m_spark.setVoltage(desiredVoltage);
  }

  // Gets the Current (in Amps) of the roller
  public double getCurrent() {
    return m_spark.getOutputCurrent();
  }
  
  // Sets the desired state of the roller
  public void requestState(GeneralRollerStates desiredState) {
    currentState = desiredState;
  }

  // Gets the current state of the roller
  public GeneralRollerStates getCurrentState() {
    return currentState;
  }
}
