
package frc.robot.subsystems.PieceHandling;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import static frc.robot.Constants.TelescopeConstants.*;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum TelescopeStates {
    StateTelescopeInit,
    StateMovingToRequestedState,
    StateTelescopeL1,
    StateTelescopeL2,
    StateTelescopeL3,
    StateTelescopeL4
    
  }

  public static TelescopeStates m_TelescopeCurrentState;
  public static TelescopeStates m_TelescopeRequestedState;

  // You may need more than one motor
  private final TalonFX m_talon = new TalonFX(kTelescopePort);
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;

  public Telescope() {
    // Misc setup goes here

    var talonFXConfigs = new TalonFXConfiguration();
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kSTelescope;
    slot0Configs.kV = kVTelescope;
    slot0Configs.kP = kPTelescope;
    slot0Configs.kI = kITelescope;
    slot0Configs.kD = kDTelescope;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kTelescopeCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = kTelescopeAcceleration;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = kTelescopeJerk;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kTelescopeClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    m_talon.getConfigurator().apply(talonFXConfigs);

    m_TelescopeCurrentState = TelescopeStates.StateTelescopeInit;
    m_TelescopeRequestedState = TelescopeStates.StateTelescopeInit;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
   
    m_talon.setPosition(0);
  }

  @Override
  public void periodic() {
    switch (m_TelescopeRequestedState) {
      case StateTelescopeInit:
        desiredPosition = 0;
        break;
      case StateTelescopeL1:
        desiredPosition = 0;
        break;
      case StateTelescopeL2:
        desiredPosition = 0;
        break;
      case StateTelescopeL3:
        desiredPosition = 0;
        break;
      case StateTelescopeL4:
        desiredPosition = 0;
        break;
    }
 
    runControlLoop();

    if (getError() < kTelescopeErrorTolerance)
      m_TelescopeCurrentState = m_TelescopeRequestedState;
    else
      m_TelescopeCurrentState = TelescopeStates.StateMovingToRequestedState;  
    }

    public void runControlLoop() {
    m_talon.setControl(request.withPosition(desiredPosition));
  }

  private double getPosition() {
    return m_talon.getPosition().getValueAsDouble();

  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }
 
  // example of a "setter" method
  public void requestState(TelescopeStates requestedState) {
    m_TelescopeRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public TelescopeStates getCurrentState() {
    return m_TelescopeCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}
