
package frc.robot.subsystems.PieceHandling;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.RollerConstants.*;

import edu.wpi.first.math.controller.PIDController;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum RollerStates {
    StateRollerOff,
    StateMovingToRequestedState,
    StateRollerOnForward,
    StateRollerOnBackward
  }

  public static RollerStates m_RollerCurrentState;
  public static RollerStates m_RollerRequestedState;

  // You may need more than one motor
  private final TalonFX m_talon = new TalonFX(kRollerPort,"Mast");
  // Unit default for TalonFX libraries is rotations
  private double desiredVoltage = 0;

  public Roller() {
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
    slot0Configs.kP = kPRoller;
    slot0Configs.kI = kIRoller;
    slot0Configs.kD = kDRoller;


    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kRollerClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    m_talon.getConfigurator().apply(talonFXConfigs);

    m_RollerCurrentState = RollerStates.StateRollerOff;
    m_RollerRequestedState = RollerStates.StateRollerOff;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
  }

  @Override
  public void periodic() {
    switch (m_RollerRequestedState) {
      case StateRollerOff:
        desiredVoltage = 0;
        break;
      case StateRollerOnForward:
        desiredVoltage = 4;
        break;
      case StateRollerOnBackward:
        desiredVoltage = -4;
        break;
    }
 
    runControlLoop();
  }
  public void runControlLoop() {
    m_talon.setVoltage(desiredVoltage);
  }

  // example of a "setter" method
  public void requestState(RollerStates requestedState) {
    m_RollerRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public RollerStates getCurrentState() {
    return m_RollerCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}

