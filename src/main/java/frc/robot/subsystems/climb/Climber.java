package frc.robot.subsystems.Climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

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
public class Climber extends SubsystemBase {

  public enum ClimberStates {
    StateNotPrimed,
    StatePrimed,
    StateUp,
    StateDownVoltage,
    StateDownPosition,
    StateMovingToRequestedState
  }

  private static ClimberStates m_ClimberCurrentState;
  private static ClimberStates m_ClimberRequestedState;

  private final TalonFX m_talonLeft = new TalonFX(kClimberLeftPort, "Mast");
  private final TalonFX m_talonRight = new TalonFX(kClimberRightPort, "Mast");

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  private double desiredPosition = 0;
  private double desiredVoltage = 0;

  private boolean leftSet = false;
  private boolean rightSet = false;

  private final Debouncer m_climberLeftDebouncer = new Debouncer(1, DebounceType.kBoth);
  private final Debouncer m_climberRightDebouncer = new Debouncer(1, DebounceType.kBoth);

  public Climber() {

    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kSClimberLeft;
    slot0Configs.kV = kVClimber;
    slot0Configs.kP = kPClimberLeft;
    slot0Configs.kI = kIClimber;
    slot0Configs.kD = kDClimber;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kClimberCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = kClimberAcceleration;
    motionMagicConfigs.MotionMagicJerk = kClimberJerk;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kClimberClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = kSensorToMechanismGearRatio;

    var currentConfigs = talonFXConfigs.CurrentLimits;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = kCurrentLimit;

    m_talonLeft.getConfigurator().apply(talonFXConfigs);

    slot0Configs.kS = kSClimberRight;
    slot0Configs.kP = kPClimberRight;

    if (kClimberClockwisePositive) {
      motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    else motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    m_talonRight.getConfigurator().apply(talonFXConfigs);

    m_ClimberCurrentState = ClimberStates.StateNotPrimed;
    m_ClimberRequestedState = ClimberStates.StateNotPrimed;

    m_talonLeft.setPosition(0);
    m_talonRight.setPosition(0);
  }

  @Override
  public void periodic() {
    switch (m_ClimberRequestedState) {
      case StateUp:
        desiredPosition = 1.6;
        desiredVoltage = 0;
        break;
      case StateDownVoltage:
        desiredPosition = 0;
        desiredVoltage = -5;
        break;
      case StateDownPosition:
        desiredPosition = -0.25;
        desiredVoltage = 0;
        break;
    }

    runControlLoop();

    if (getErrorLeft() < kClimberErrorTolerance && getErrorRight() < kClimberErrorTolerance)
      m_ClimberCurrentState = m_ClimberRequestedState;
    else
      m_ClimberCurrentState = ClimberStates.StateMovingToRequestedState;

  }

  private void runControlLoop() {
    if (desiredVoltage != 0) {
        if (!leftSet) {
            m_talonLeft.setVoltage(desiredVoltage);
        } else {
            m_talonLeft.setVoltage(0);
        }
        
        if (!rightSet) {
            m_talonRight.setVoltage(desiredVoltage);
        } else {
            m_talonRight.setVoltage(0);
        }
    } else {
        var controlRequest = request.withPosition(desiredPosition).withEnableFOC(true);
        m_talonLeft.setControl(controlRequest);
        m_talonRight.setControl(controlRequest);
    }
}

  private double getPositionLeft() {
    return m_talonLeft.getPosition().getValue();
  }

  public double getErrorLeft() {
    if (getRequestedState() != ClimberStates.StateDownVoltage)
      return Math.abs(getPositionLeft() - desiredPosition);
    else
      return m_climberLeftDebouncer.calculate(Math.abs(m_talonLeft.getTorqueCurrent().getValueAsDouble()) < kCurrentLimitThreshold && !leftSet && getPositionLeft() < 1.9) ? 0 : kClimberErrorTolerance + 1;
  }

  private double getPositionRight() {
    return m_talonRight.getPosition().getValue();
  }

  public double getErrorRight() {
    if (getRequestedState() != ClimberStates.StateDownVoltage) {
      return Math.abs(getPositionRight() - desiredPosition);
    } else {
      return m_climberRightDebouncer.calculate(m_talonRight.getTorqueCurrent().getValueAsDouble() < kCurrentLimitThreshold && !rightSet && getPositionRight() < 1.9) ? 0 : kClimberErrorTolerance + 1;
    }
  }

  public void zeroClimber() {
    m_talonLeft.setPosition(0);
    m_talonRight.setPosition(0);
  }

  public void requestState(ClimberStates requestedState) {
    m_ClimberRequestedState = requestedState;
  }

  public ClimberStates getCurrentState() {
    return m_ClimberCurrentState;
  }

  public ClimberStates getRequestedState() {
    return m_ClimberRequestedState;
  }
}
