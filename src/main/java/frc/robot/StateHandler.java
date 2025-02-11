package frc.robot;

import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import static frc.robot.Constants.PivotConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PieceHandling.Elevator;
import frc.robot.subsystems.PieceHandling.Pivot;
import frc.robot.subsystems.PieceHandling.Wrist;
import frc.robot.subsystems.PieceHandling.Elevator.ElevatorStates;
import frc.robot.subsystems.PieceHandling.Pivot.PivotStates;
import frc.robot.subsystems.PieceHandling.Wrist.WristStates;

/*
 * 
 * This class should handle ALL transitions between states. It is NOT safe to transition from the L2 setpoint to the intake setpoint,
 * As demonstrated by yesterday's fiasco. We need a way to control which subsystems move in which order. To do this, we should be using getError() on the
 * Positional values of the susbystem to find how far we are from the setpoint. If it's within tolerance, we can start the next motion. -NS
 * 
 * Yes, it is a Subsystem technically--- it seems to be the easiest way to call something periodically without throwing it in robotcontainer.
 * 
 * This system should also help in the case we want to restrict movement while extended.....
 * 
 */


public class StateHandler extends SubsystemBase{
    public RobotStates currentState = RobotStates.StateInit;
    public RobotStates requestedState = RobotStates.StateInit;

    private Elevator m_Elevator;
    private Wrist m_Wrist;
    private Pivot m_Pivot;

    public enum RobotStates {

        StateBetweenStates,
        StateInit,
        StateZero,
        StateCoralStation,
        StateL1,
        StateL2,
        StateL3,
        StateL4,
    }

    public StateHandler(Elevator elevator, Wrist wrist, Pivot pivot){ // include subsystems as argument

        m_Elevator=elevator;
        m_Wrist = wrist;
        m_Pivot = pivot;    

    }
    
    
    
    @Override
    public void periodic(){

        switch(requestedState){

            case StateInit:
                requestedState=RobotStates.StateZero;
                break;
            case StateZero:
                m_Wrist.requestState(WristStates.StateZero); // just making the assumption that wrist must retract before the other subsystems 
                if(m_Wrist.getCurrentState() == WristStates.StateZero){ // the wrist is in movingtorquestedstate when NOT at goal...

                    m_Elevator.requestState(ElevatorStates.StateZero);
                    m_Pivot.requestState(PivotStates.StateZero);

                }
                
                break;
            case StateCoralStation:
                m_Wrist.requestState(WristStates.StateCoralStation); // just making the assumption that wrist must retract before the other subsystems 
                if(m_Wrist.getCurrentState() == WristStates.StateCoralStation){ // the wrist is in movingtorquestedstate when NOT at goal...

                    m_Elevator.requestState(ElevatorStates.StateCoralStation);
                    m_Pivot.requestState(PivotStates.StateCoralStation);

                }
                
                break;
            case StateL2:
                m_Elevator.requestState(ElevatorStates.StateL2); // making the assumption it's the opposite as going to zero...
                if(m_Elevator.getCurrentState() == ElevatorStates.StateL2 ){// the elevator is in movingtorquestedstate when NOT at goal...
                    // GetCurrentState is better than directly checking for error because it ensures we're both within tolerance AND in the RIGHt state, so this if statement won't pass if we're in the right spot for L1 for example.
                    // This approach is also sick if you want to use transitional states because you can call request to the transition, see if the transition is complete, and then execute the rest.
                    m_Wrist.requestState(WristStates.StateL2);
                    m_Pivot.requestState(PivotStates.StateL2);

                }
                break;

        }


        if(getStateReached()){

            currentState=requestedState;

        }
        else{
            currentState = RobotStates.StateBetweenStates;

        }
    }
    public void requesstRobotState(RobotStates changeTo){

        requestedState=changeTo;

    }

    public boolean getStateReached(){

        if(m_Wrist.getError() < kWristErrorTolerance && m_Pivot.getError() < kPivotErrorTolerance && m_Elevator.getError()< kElevatorErrorTolerance ){
            return true;
        }
        else {
            return false;
        }

    }
}
