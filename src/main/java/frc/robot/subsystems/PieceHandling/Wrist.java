package frc.robot.subsystems.PieceHandling;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PieceHandling.Elevator.ElevatorStates;

import static frc.robot.Constants.WristConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.WristConstants.*;
public class Wrist extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum WristStates { // positive voltage moves downwards
    StateInit,
    StateZero, //intake position / default position 
    StateL1,
    StateL2,
    StateL3,
    StateL4,
    StateIntake,
    StateBarge,
    StateProcessor,
    StateL2Algae,
    StateL3Algae,
    StateLollipop,
    StateMoveToRequestedState,
    StateStation,
    StateL4Tuck,
    StateClimb
  }

  public static WristStates m_WristCurrentState;
  public static WristStates m_WristRequestedState;
  // You may need more than one motor
  private final TalonFX m_talon = new TalonFX(kWrist1Port, "Mast");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
//  private final TalonFX m_WristTalon2 = new TalonFX(kWrist2Port,"Mast");
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;
 // private DoubleSupplier supplier = new DoubleSupplier() 
  public static GenericEntry motorVelocity = Shuffleboard.getTab("Wrist").add("Velocity", 0.0).getEntry();
  public static GenericEntry desiredPositionLog = Shuffleboard.getTab("Wrist").add("desiredPos", 0).getEntry();
  public static GenericEntry motorPosition = Shuffleboard.getTab("Wrist").add("position", 0.0).getEntry();

  public static GenericEntry errorLog = Shuffleboard.getTab("Wrist").add("error", 0.0).getEntry();
  
  public Wrist() {
    // Misc setup goes here

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kSWrist;
    slot0Configs.kG = kGWrist;// kGWrist;

    slot0Configs.kV = 1.0;
    slot0Configs.kA = 0.075;
    slot0Configs.kP = 10;// kPWrist;
    slot0Configs.kI = 0.1;
    slot0Configs.kD = 0.25;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 60;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = 1250;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = 2500;
    
    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kWristClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_talon.getConfigurator().apply(talonFXConfigs);
    //m_WristTalon2.getConfigurator().apply(talonFXConfigs);
    m_WristCurrentState = WristStates.StateInit;
    m_WristRequestedState = WristStates.StateInit;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    m_talon.setPosition(0);
    
   // m_WristTalon2.setPosition(0);
  }


  public double getVelocity(){

    return m_talon.getVelocity().getValueAsDouble();

  }

  @Override
  public void periodic() {
    switch (m_WristRequestedState) {
      case StateInit:
        desiredPosition = 0;
        break;
      case StateZero:
        desiredPosition = 0;
        break;
      case StateStation:
        desiredPosition = 0.25/kWristGearRatio;
        break;
      case StateL1:
        desiredPosition = .7;
        break;
      case StateL2:
        desiredPosition = 0.7;
        break;
      case StateL3:
        desiredPosition = 0.6336;//-0.21891;
        break;
      case StateL4:

        desiredPosition = 0.54; //(0.68750-0.21891) + (1/18);

        break;
      case StateIntake:
        desiredPosition = 0;
        break;
      case StateBarge:
        desiredPosition = 21/kWristGearRatio;
        break;
      case StateProcessor:
        desiredPosition = 0.13;
        break;
      case StateL2Algae:
        desiredPosition = 0.19;
        break;
      case StateL3Algae:
        desiredPosition = 0.16;
        break;
      case StateLollipop:
        desiredPosition = 0.2/kWristGearRatio;
        break;
      case StateClimb:
        desiredPosition=0.16;
        break;
      case StateL4Tuck:
        desiredPosition=(25/kWristGearRatio );
        break;
      }
    desiredPosition*=kWristGearRatio;
    runControlLoop();
    errorLog.setDouble(getError());
 // 4.2 wrist 14 pivot
    if (getError() < kWristErrorTolerance)
      m_WristCurrentState = m_WristRequestedState;
     else
     m_WristCurrentState = WristStates.StateMoveToRequestedState;  
  }

  public void runControlLoop() {
   // MotionMagicVoltage voltage = request.withPosition(desiredPosition);
    
    m_talon.setControl(request.withPosition(desiredPosition));
    motorVelocity.setDouble(m_talon.getVelocity().getValueAsDouble());
    //desiredPositionLog.setDouble(desiredPosition);
    motorPosition.setDouble(m_talon.getPosition().getValueAsDouble());

  //  m_WristTalon2.setControl(request.withPosition(desiredPosition));
  }

  public double getPosition() {
    return m_talon.getPosition().getValueAsDouble();
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }
 
  // example of a "setter" method
  public void requestState(WristStates requestedState) {
    m_WristRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public WristStates getCurrentState() {
    return m_WristCurrentState;
  }

  public WristStates getRequestedState(){

    return m_WristRequestedState;

  }
  // misc methods go here, getters and setters should follow above format
}
