package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import static frc.robot.Constants.LEDConstants.*; 

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.crte.phoenix.led.ColorFlowAnimation.Direction;

public class LED extends SubsystemBase {
  // Enum representing all of the states the subsystem can be in
  public enum LEDStates {
    StateOff,
    StateWhite,
    StateYellow,
    StatePink,
    StateRainbow
  }

  public static LEDStates m_LEDCurrentState;
  public static LEDStates m_LEDRequestedState;

  CANdle candle;

  public LED() {
    try {
      m_LEDRequestedState = LEDStates.StateOff;
      candle = new CANdle(0); // set to the actual ID
      CANdleConfiguration config = new CANdleConfiguration();
      config.stripType = LEDStripType.RGB;
      config.brightnessScalar = 1.0;
      candle.configAllSettings(config);
    }
    catch (Exception e) {}

  }

  @Override
  public void periodic() {
    try {
      switch (m_LEDRequestedState) {
        case StateOff:
          // Turn off LED
          candle.setLEDs(0, 0, 0);
          break;
        case StateWhite:
          // Set the LED to white
          candle.setLEDs(255, 255, 255);
          break;
        case StateYellow:
          // Set the LED to yellow
          candle.setLEDs(249, 166, 3); // Change to a different yellow maybe
          break;
        case StatePink:
          candle.setLEDs(255, 16, 240);
          break;
        case StateRainbow:
          RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
          candle.animate(rainbowAnim);
          break;
      }

      m_LEDCurrentState = m_LEDRequestedState;

      runControlLoop();
    }
    catch (Exception e) {}
  }
  public void runControlLoop() {}

 
  // example of a "setter" method
  public void requestState(LEDStates requestedState) {
    try {
      m_LEDRequestedState = requestedState;
      System.out.println("dummy");
    }
    catch (Exception e) {}
  }
  // example of a "getter" method
  public LEDStates getCurrentState() {
    return m_LEDCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}
