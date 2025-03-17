
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.Currency;

import org.opencv.core.Mat;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum ClimberStates {
    StateClimberOff,
    StateMovingToRequestedState,
    StateClimberOnForward,
    StateClimberOnBackward,

  }

  public static ClimberStates m_ClimberCurrentState;
  public static ClimberStates m_ClimberRequestedState;

  // You may need more than one motor
  private final TalonFX m_talon = new TalonFX(kClimberPort,"Mast");
  // private GenericEntry stateLog = Shuffleboard.getTab("Climber").addString("Climber State", "blah").;
  public static GenericEntry state = Shuffleboard.getTab("Climber").add("State of Climber", "init").getEntry();
  public static GenericEntry stateRequested = Shuffleboard.getTab("Climber").add("Req. State of Climber", "init").getEntry();
  public static GenericEntry currentLog = Shuffleboard.getTab("Climber").add("current",0.0).getEntry();
  public static GenericEntry veloLog = Shuffleboard.getTab("Climber").add("velocity",0.0).getEntry();

  public static GenericEntry posLog = Shuffleboard.getTab("Climber").add("position" , 0.0).getEntry();

  // Unit default for TalonFX libraries is rotations
  private double desiredVoltage = 0;

  public Climber() {
    // Misc setup goes here

    var talonFXConfigs = new TalonFXConfiguration();
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second


    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable=true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit=120;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit=120;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable=true;
    if (kClimberClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    m_talon.getConfigurator().apply(talonFXConfigs);

    m_ClimberCurrentState = ClimberStates.StateClimberOff;
    m_ClimberRequestedState = ClimberStates.StateClimberOff;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
  }

  @Override
  public void periodic() {
    switch (m_ClimberRequestedState) {
      case StateClimberOff:
        desiredVoltage = 0;
        break;
      case StateClimberOnForward:
        desiredVoltage = 5;
        break;
      case StateClimberOnBackward:
        desiredVoltage = -5;
        break;

      }

 
    runControlLoop();
    // state.setString(m_ClimberCurrentState.toString());
    stateRequested.setString(m_ClimberRequestedState.toString());
  }



  public void runControlLoop() {
    currentLog.setDouble(m_talon.getSupplyCurrent().getValueAsDouble());
    veloLog.setDouble(m_talon.getVelocity().getValueAsDouble());
    posLog.setDouble(m_talon.getPosition().getValueAsDouble());



    if(desiredVoltage == 0){
      m_talon.setVoltage(desiredVoltage);


    }else if(m_talon.getPosition().getValueAsDouble() < -380 && desiredVoltage < 0){

      m_talon.setVoltage(0);

    }
    else{
      m_talon.setVoltage(desiredVoltage);

    }

  }

  // example of a "setter" method
  public void requestState(ClimberStates requestedState) {
    m_ClimberRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public ClimberStates getCurrentState() {
    return m_ClimberCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}

