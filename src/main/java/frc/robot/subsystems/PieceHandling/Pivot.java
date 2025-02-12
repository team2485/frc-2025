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
    StateProcessor,
    StateBarge,
    StateLowAlgae,
    StateHighAlgae,
    StateLollipop,
    StateMoveToRequestedState,
    StateStation
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

  public Pivot() {
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
    slot0Configs.kS = kSPivot;
    slot0Configs.kG = kGPivot;// kGPivot;

    slot0Configs.kV = kVPivot;
    slot0Configs.kA = kAPivot;
    slot0Configs.kP = kPPivot;// kPPivot;
    slot0Configs.kI = kIPivot;
    slot0Configs.kD = kDPivot;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kPivotCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = kPivotAcceleration;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = 0;
    
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
      case StateStation:
        desiredPosition = 0.02;
        break;
      case StateL1:
        desiredPosition = 0.05;
        break;
      case StateL2:
        desiredPosition = 0;
        break;
      case StateL3:
        desiredPosition = 0.1;
        break;
      case StateL4:
        desiredPosition = 0.1;
        break;
      case StateProcessor:
        desiredPosition = 0.1;
        break;
      case StateBarge:
        desiredPosition = 0.1;
        break;
      case StateLowAlgae:
        desiredPosition = 0.1;
        break;
      case StateHighAlgae:
        desiredPosition = 0.1;
        break;
      case StateLollipop:
        desiredPosition = 0.1;
        break;
    }
    desiredPosition*=kPivotGearRatio;
    runControlLoop();

    // if (getError() < kPivotErrorTolerance)
    m_PivotCurrentState = m_PivotRequestedState;
    // else
    // m_PivotCurrentState = PivotStates.StateMoveToRequestedState;  
  }

  public void runControlLoop() {
   // MotionMagicVoltage voltage = request.withPosition(desiredPosition);
    
    m_talon.setControl(request.withPosition(desiredPosition));
    motorVoltage.setDouble(m_talon.getVelocity().getValueAsDouble());
    desiredPositionLog.setDouble(desiredPosition);
  //  m_PivotTalon2.setControl(request.withPosition(desiredPosition));
  }

  private double getPosition() {
    return m_talon.getPosition().getValueAsDouble();
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

  // misc methods go here, getters and setters should follow above format
}
