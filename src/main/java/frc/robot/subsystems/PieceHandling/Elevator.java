package frc.robot.subsystems.PieceHandling;

// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*; 

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class Elevator extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum ElevatorStates {
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
  }

  public static ElevatorStates m_ElevatorCurrentState;
  public static ElevatorStates m_ElevatorRequestedState;

  // You may need more than one motor
  private final TalonFX m_elevatorTalon1 = new TalonFX(kElevator1Port);
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  private final TalonFX m_elevatorTalon2 = new TalonFX(kElevator2Port);
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;

  public Elevator() {
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
    slot0Configs.kS = kSElevator;
    slot0Configs.kV = kVElevator;
    slot0Configs.kP = kPElevator;
    slot0Configs.kI = kIElevator;
    slot0Configs.kD = kDElevator;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kElevatorCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = kElevatorAcceleration;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = kElevatorJerk;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kElevatorClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    m_talon.getConfigurator().apply(talonFXConfigs);

    m_ElevatorCurrentState = ElevatorStates.StateInit;
    m_ElevatorRequestedState = ElevatorStates.StateInit;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    m_talon.setPosition(0);
  }

  @Override
  public void periodic() {
    switch (m_ElevatorRequestedState) {
      case StateInit:
        desiredPosition = 0;
        break;
      case StateZero:
        desiredPosition = 0;
        break;
      case StateL1:
        desiredPosition = 1;
        break;
      case StateL2:
        desiredPosition = 2;
        break;
      case StateL3:
        desiredPosition = 3;
        break;
      case StateL4:
        desiredPosition = 4;
        break;
      case StateProcessor:
        desiredPosition = 0.25;
        break;
      case StateBarge:
        desiredPosition = 6;
        break;
      case StateLowAlgae:
        desiredPosition = 1.5;
        break;
      case StateHighAlgae:
        desiredPosition = 2.5;
        break;
      case StateLollipop:
        desiredPosition = 0.35;
        break;
    }
 
    runControlLoop();

    if (getError() < kElevatorErrorTolerance)
      m_ElevatorCurrentState = m_ElevatorRequestedState;
    else
      m_ElevatorCurrentState = ElevatorStates.StateMovingToRequestedState;  
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
  public void requestState(ElevatorStates requestedState) {
    m_ElevatorRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public ElevatorStates getCurrentState() {
    return m_ElevatorCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}
