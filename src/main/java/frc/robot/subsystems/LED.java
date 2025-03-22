package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import static frc.robot.Constants.LEDConstants.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.crte.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class LED extends SubsystemBase {
  // Enum representing all of the states the subsystem can be in
  public enum LEDStates {
    StateOff,
    StateWhite,
    StateYellow,
    StatePink,
    StateRainbow,
    StateBlueAnim
  }

  public static LEDStates m_LEDCurrentState;
  public static LEDStates m_LEDRequestedState;
  private static boolean runningAnimation;

  CANdle candle;

  public LED() {
    m_LEDRequestedState = LEDStates.StateOff;
    m_LEDCurrentState = LEDStates.StateOff;
    candle = new CANdle(15, "Mast"); // set to the actual ID
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);
    runningAnimation = false;
  }

  @Override
  public void periodic() {
    switch (m_LEDRequestedState) {
      case StateOff:
        // Turn off LED
        runningAnimation = false;
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
        break;
      case StateWhite:
        // Set the LED to white
        runningAnimation = false;
        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 255);
        break;
      case StateYellow:
        // Set the LED to yellow
        runningAnimation = false;
        candle.clearAnimation(0);
        candle.setLEDs(250, 242, 3); // Change to a different yellow maybe
        break;
      case StatePink:
        runningAnimation = false;
        candle.clearAnimation(0);
        candle.setLEDs(255, 16, 240);
        break;
      case StateRainbow:
        if(!runningAnimation) {
          candle.clearAnimation(0);
          RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.75, 68);
          candle.animate(rainbowAnim);
          runningAnimation = true;
        }
        break;
      case StateBlueAnim:
        // Blue streak animation
        if(!runningAnimation) {
          candle.clearAnimation(0);
          // ColorFlowAnimation blue2 = new ColorFlowAnimation(26, 152, 218, 0, 0.5, 64, Direction.Forward);
          LarsonAnimation blue = new LarsonAnimation(26, 152, 218, 0, 0.25, 68, BounceMode.Center, 5);
          candle.animate(blue);
          runningAnimation = true;
        }
        break;
    }
    // autos will have alliance color for LEDs
    // green LEDs for scoring in processor

    m_LEDCurrentState = m_LEDRequestedState;

    runControlLoop();
  }
    public void runControlLoop() {
    }

 
  // example of a "setter" method
  public void requestState(LEDStates requestedState) {
    m_LEDRequestedState = requestedState;
  }
  // example of a "getter" method
  public LEDStates getCurrentState() {
    return m_LEDCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}