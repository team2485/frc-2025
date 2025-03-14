// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// // import static frc.robot.Constants.CANdleConstants.*; 

// // import com.ctre.phoenix.led.*;
// // import com.ctre.phoenix.led.CANdle.LEDStripType;
// // import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// // import com.crte.phoenix.led.ColorFlowAnimation.Direction;

// public class CANdle extends SubsystemBase {
//   // Enum representing all of the states the subsystem can be in
//   public enum CANdleStates {
//     StateOff,
//     StateWhite,
//     StateYellow
//   }

//   public static CANdleStates m_CANdleCurrentState;
//   public static CANdleStates m_CANdleRequestedState;


//   public CANdle() {
//     m_CANdleRequestedState = CANdleStates.StateOff;
//   }

//   @Override
//   public void periodic() {
//     switch (m_CANdleRequestedState) {
//       case StateOff:
//         candle.config.
//         break;
//       case StateWhite:
//         // Set the LEDs to white
//         break;
//       case StateYellow:
//         // Set the LEDs to yellow
//         break;
//     }

//     m_CANdleCurrentState = m_CANdleRequestedState;

//     runControlLoop();
//   }
//     public void runControlLoop() {}

 
//   // example of a "setter" method
//   public void requestState(CANdleStates requestedState) {
//     m_CANdleRequestedState = requestedState;
//   }
 
//   // example of a "getter" method
//   public CANdleStates getCurrentState() {
//     return m_CANdleCurrentState;
//   }

//   // misc methods go here, getters and setters should follow above format
// }
