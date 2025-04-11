package frc.robot.subsystems.PieceHandling;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PivotConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.PivotConstants.*;
public class Pivot extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum PivotStates { // positive voltage moves downwards
    StateInit,
    StateZero, //intake position / default position 
    StateL1,
    StateL2,
    StateL3,
    StateL4,
    StateL4Transition,
    StateProcessor,
    StateBarge,
    StateL2Algae,
    StateL3Algae,
    StateLollipop,
    StateMoveToRequestedState,
    StateStation,
    StateClimb,
    StateIntake
  }



  public static PivotStates m_PivotCurrentState;
  public static PivotStates m_PivotRequestedState;
  // You may need more than one motor
  private final TalonFX m_talon = new TalonFX(kPivot1Port, "Mast");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
//  private final TalonFX m_PivotTalon2 = new TalonFX(kPivot2Port,"Mast");
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;
 // private DoubleSupplier supplier = new DoubleSupplier() 
  public static GenericEntry motorVoltage = Shuffleboard.getTab("Pivot").add("velocity", 0.0).getEntry();
  public static GenericEntry desiredPositionLog = Shuffleboard.getTab("Pivot").add("desiredPos", 0).getEntry();
  public static GenericEntry motorPosition = Shuffleboard.getTab("Pivot").add("position", 0.0).getEntry();

  public Pivot() {
    // Misc setup goes here

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 20;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kSPivot;
    slot0Configs.kG = 0;// kGPivot;
    slot0Configs.kV = 0.25;
    slot0Configs.kA = 0.01;
    slot0Configs.kP = 2;// kPPivot;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.2;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = 600;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = 1800;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kPivotClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_talon.getConfigurator().apply(talonFXConfigs);
    //m_PivotTalon2.getConfigurator().apply(talonFXConfigs);
    m_PivotCurrentState = PivotStates.StateInit;
    m_PivotRequestedState = PivotStates.StateInit;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    m_talon.setPosition(0);
    
   // m_PivotTalon2.setPosition(0);
  }

  @Override
  public void periodic() {
    switch (m_PivotRequestedState) {
      case StateInit:
        desiredPosition = 0;
        break;

      case StateZero:
        desiredPosition = 0;
        break;
      case StateClimb:
        desiredPosition = 0.172;
        break;
      case StateIntake:
        desiredPosition = 0;
        break;
      case StateStation:
        desiredPosition = 0.02;
        break;
      case StateL4Transition:
        desiredPosition = 0.125;
      case StateL1:
        desiredPosition = 0.03;
        break;
      case StateL2:
        desiredPosition = 0.03;
        break;
      case StateL3:
        desiredPosition = 0;
        break;
      case StateL4:
        desiredPosition = 0.34027;
        break;
      case StateProcessor:
        desiredPosition = 0.02;
        break;
      case StateBarge:
        desiredPosition = 0.4178;
        break;
      case StateL2Algae:
        desiredPosition = 0.172;
        break;
      case StateL3Algae:
        desiredPosition = 0.172;
        break;
      case StateLollipop:
        desiredPosition = 0.1;
        break;
    }
    desiredPosition*=kPivotGearRatio;
    runControlLoop();

    if (getError() < kPivotErrorTolerance)
      m_PivotCurrentState = m_PivotRequestedState;
    else
      m_PivotCurrentState = PivotStates.StateMoveToRequestedState;  
  }

  public void runControlLoop() {
   // MotionMagicVoltage voltage = request.withPosition(desiredPosition);
    
    m_talon.setControl(request.withPosition(desiredPosition));
    motorVoltage.setDouble(m_talon.getVelocity().getValueAsDouble());
    //desiredPositionLog.setDouble(desiredPosition);
    motorPosition.setDouble(m_talon.getPosition().getValueAsDouble());
  //  m_PivotTalon2.setControl(request.withPosition(desiredPosition));
  }

  public double getPosition() {
    return m_talon.getPosition().getValueAsDouble();
  }
  public double getPositionWithRatio(){
    return m_talon.getPosition().getValueAsDouble()/kPivotGearRatio;

  }
  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }
 
  // example of a "setter" method
  public void requestState(PivotStates requestedState) {
    m_PivotRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public PivotStates getCurrentState() {
    return m_PivotCurrentState;
  }
  public PivotStates getRequestedState(){

    return m_PivotRequestedState;

  }
  // misc methods go here, getters and setters should follow above format
}
