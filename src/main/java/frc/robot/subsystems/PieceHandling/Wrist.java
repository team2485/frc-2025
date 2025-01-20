package frc.robot.subsystems.PieceHandling;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


/*

    THIS CLASS DOES NOT WORK YET, THIS IS A TEMPLATE THAT I WROTE, 
    PLEASE CHANGE THE PARAMAETERS ACCORDINGLY!
    TODO:

*/
public class Wrist extends SubsystemBase {
  
  public enum WristStates {
    StateOff,
    StateOuttake,
    StateIntake,
    StatePivotL1,
    StatePivotL2,
    StatePivotL3,
    StatePivotL4,
    StateCoralIntake,
    StateLowAlgae,
    StateHighAlgae,
    StateLollipopAlgae,
    StateProcessor,
    StateNet,
    StateOuttakeIntake,
    StateMovingToRequestedState
  }

  public static PivotStates m_PivotCurrentState;
  public static PivotStates m_PivotRequestedState;

  GenericEntry armPosition;

  private final TalonFX m_talon = new TalonFX(kPivotPort, "Mast");
  DoubleSupplier angle; 
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  private double desiredPosition = 0;

  public Pivot(DoubleSupplier angle) {

    this.angle = angle;
    
    armPosition = Shuffleboard.getTab("Swerve").add("ArmPosition", 0).getEntry();


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
    slot0Configs.kV = kVPivot;
    slot0Configs.kP = kPPivot;
    slot0Configs.kI = kIPivot;
    slot0Configs.kD = kDPivot;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kPivotCruiseVelocity;

    motionMagicConfigs.MotionMagicAcceleration = kPivotAcceleration;

    motionMagicConfigs.MotionMagicJerk = kPivotJerk;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kPivotClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = kSensorToMechanismGearRatio;

    var currentConfigs = talonFXConfigs.CurrentLimits;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs .StatorCurrentLimit = kCurrentLimit;

    m_talon.getConfigurator().apply(talonFXConfigs);

    m_PivotCurrentState = PivotStates.StateOff;
    m_PivotRequestedState = PivotStates.StateOff;

    m_talon.setPosition(0);
  }

  @Override
  public void periodic() {

    armPosition.setDouble(getError());

    switch (m_PivotRequestedState) {
      case StateOuttake:
      case StateIntake:
        desiredPosition = .06;
        break;
      case StatePivotL1:
        desiredPosition = 0.15;
        break;
      case StatePivotL2:
        desiredPosition = 0.20;
        break;
      case StatePivotL3:
        desiredPosition = 0.30;
        break;
      case StatePivotL4:
        desiredPosition = 0.35;
        break;
      case StateCoralIntake:
        desiredPosition = 0.05;
        break;
      case StateLowAlgae:
        desiredPosition = 0.10;
        break;
      case StateHighAlgae:
        desiredPosition = 0.25;
        break;
      case StateLollipopAlgae:
        desiredPosition = 0.40;
        break;
      case StateProcessor:
        desiredPosition = 0.50;
        break;
      case StateNet:
        desiredPosition = 0.60;
        break;
      case StateOuttakeIntake:
        desiredPosition = 0.15;
        break;
      case StateOff:
      case StateMovingToRequestedState:
        desiredPosition = 0;
        break;
    }
 
    runControlLoop();

    if (getError() < kPivotErrorTolerance)
      m_PivotCurrentState = m_PivotRequestedState;
    else
      m_PivotCurrentState = PivotStates.StateMovingToRequestedState;  
  }

  public void runControlLoop() {
    m_talon.setControl(request.withPosition(desiredPosition));
  }

  private double getPosition() {
    return m_talon.getPosition().getValue();
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }

  public void requestState(PivotStates requestedState) {
    m_PivotRequestedState = requestedState;
  }
 
  public PivotStates getCurrentState() {
    return m_PivotCurrentState;
  }

}
