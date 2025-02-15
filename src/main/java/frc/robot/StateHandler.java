package frc.robot;

import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    public RobotStates requestedState = RobotStates.StateCoralStationInit;


    private Elevator m_Elevator;
    private Wrist m_Wrist;
    private Pivot m_Pivot;


    public enum RobotStates {

        StateBetweenStates,
        StateInit,
        StateZero,
        StateCoralStationInit,
        StateCoralStationTransition,
        StateCoralStationTransition2,
        StateCoralStationFinal,
        StateL1,
        StateL2,
        StateL2Init,
        StateL2Finished,
        StateL2Transition3,
        StateL2Transition,
        StateL2Transition2,
        StateL2WristTransition,
        StateL3Init,
        StateL3Finished,
        StateL3Transition3,
        StateL3Transition,
        StateL3Transition2,
        StateL3WristTransition,
        StateL3,
        StateL4Init,

        
        StateL4RetractTransition1,
        StateL4RetractInit2,
        StateL4RetractTransition2,
        StateL4RetractTransition3,
        StateL4RetractInit3,
        StateL4RetractFinal,
        StateL4Finished,
        StateL4Transition3,
        StateL4Transition,
        StateL4Transition2,
        StateL4WristTransition,
        StateL4,
        StateL4RetractInit,

        StateL2Algae,
        StateL3Algae,
        StateLollipop
    }

    public StateHandler(Elevator elevator, Wrist wrist, Pivot pivot){ // include subsystems as argument

        m_Elevator = elevator;
        m_Wrist = wrist;
        m_Pivot = pivot;    

    }
    
    GenericEntry state = Shuffleboard.getTab("Elevator").add("state", "").getEntry();
    
    @Override
    public void periodic(){

        switch(currentState){

            case StateInit:
                currentState=RobotStates.StateCoralStationInit;
                break;
            case StateZero:
                m_Wrist.requestState(WristStates.StateZero); // just making the assumption that wrist must retract before the other subsystems 
                if(m_Wrist.getCurrentState() == WristStates.StateZero){ // the wrist is in movingToRequestedState when NOT at goal...

                    m_Elevator.requestState(ElevatorStates.StateZero);
                    m_Pivot.requestState(PivotStates.StateZero);

                }
                
                break;
            case StateCoralStationInit:
                m_Elevator.requestState(ElevatorStates.StateStation); // just making the assumption that wrist must retract before the other subsystems 
                currentState = RobotStates.StateCoralStationTransition;
                break;
            case StateCoralStationTransition:
                
                if(m_Elevator.getCurrentState() == ElevatorStates.StateStation) { // the wrist is in movingToRequestedState when NOT at goal...

                    m_Wrist.requestState(WristStates.StateStation);
                    m_Pivot.requestState(PivotStates.StateStation);
                    currentState = RobotStates.StateCoralStationTransition2;
                }
                break;
            case StateCoralStationTransition2:
                if(m_Wrist.getCurrentState() == WristStates.StateStation && m_Pivot.getCurrentState() == PivotStates.StateStation) {
                    currentState = RobotStates.StateCoralStationFinal;
                }
                break;
            case StateCoralStationFinal:
                if(requestedState != RobotStates.StateCoralStationInit){
                    currentState=requestedState;
                }
                break;
            case StateL2Init:
                m_Elevator.requestState(ElevatorStates.StateL2); // making the assumption it's the opposite as going to zero...
                currentState = RobotStates.StateL2Transition;
                break;  
            case StateL2Transition:
                if(m_Elevator.getCurrentState() == ElevatorStates.StateL2){// the elevator is in movingToRequestedState when NOT at goal...
                    // GetCurrentState is better than directly checking for error because it ensures we're both within tolerance AND in the RIGHT state, so this if statement won't pass if we're in the right spot for L1 for example.
                    // This approach is also sick if you want to use transitional states because you can call request to the transition, see if the transition is complete, and then execute the rest.
                    m_Wrist.requestState(WristStates.StateL2);
                    
                    currentState=RobotStates.StateL2Transition2;
                }
                break;
            case StateL2Transition2:
                if(m_Wrist.getCurrentState() == WristStates.StateL2) {
                    currentState=RobotStates.StateL2Transition3;
                    m_Pivot.requestState(PivotStates.StateL2);
                }
                break;
            case StateL2Transition3:
                if(m_Wrist.getCurrentState() == WristStates.StateL2 && m_Pivot.getCurrentState() == PivotStates.StateL2){

                    currentState=RobotStates.StateL2Finished;

                }
                break;
            case StateL2Finished:
                if(requestedState==RobotStates.StateCoralStationInit) {
                    currentState = RobotStates.StateCoralStationInit;
                }
                break;
                
            case StateL3Init:
                m_Elevator.requestState(ElevatorStates.StateL3); // making the assumption it's the opposite as going to zero...
                currentState = RobotStates.StateL3Transition;
                break;

            case StateL3Transition:
                if(m_Elevator.getCurrentState() == ElevatorStates.StateL3){// the elevator is in movingToRequestedState when NOT at goal...
                    // GetCurrentState is better than directly checking for error because it ensures we're both within tolerance AND in the RIGHT state, so this if statement won't pass if we're in the right spot for L1 for example.
                    // This approach is also sick if you want to use transitional states because you can call request to the transition, see if the transition is complete, and then execute the rest.
                    m_Wrist.requestState(WristStates.StateL3);
                    
                    currentState=RobotStates.StateL3Transition2;
                }
                break;
            case StateL3Transition2:
                if(m_Wrist.getCurrentState() == WristStates.StateL3) {
                    currentState=RobotStates.StateL3Transition3;
                    m_Pivot.requestState(PivotStates.StateL3);
                }
                break;
            case StateL3Transition3:
                if(m_Wrist.getCurrentState() == WristStates.StateL3 && m_Pivot.getCurrentState() == PivotStates.StateL3){

                    currentState=RobotStates.StateL3Finished;

                }
                break;
            case StateL3Finished:
                if(requestedState==RobotStates.StateCoralStationInit) {
                    currentState = RobotStates.StateCoralStationInit;
                }
                break;
            
            case StateL4Init:
                m_Elevator.requestState(ElevatorStates.StateL4); // making the assumption it's the opposite as going to zero...
                currentState = RobotStates.StateL4Transition;
                break;

            case StateL4Transition:
                if(m_Elevator.getCurrentState() == ElevatorStates.StateL4){// the elevator is in movingToRequestedState when NOT at goal...
                    // GetCurrentState is better than directly checking for error because it ensures we're both within tolerance AND in the RIGHT state, so this if statement won't pass if we're in the right spot for L1 for example.
                    // This approach is also sick if you want to use transitional states because you can call request to the transition, see if the transition is complete, and then execute the rest.
                    m_Wrist.requestState(WristStates.StateL4);
                    m_Pivot.requestState(PivotStates.StateL4);
                    
                    currentState=RobotStates.StateL4Transition2;
                }
                break;
            case StateL4Transition2:
                if(m_Wrist.getCurrentState() == WristStates.StateL4 && m_Pivot.getCurrentState() == PivotStates.StateL4){

                    currentState=RobotStates.StateL4Finished;

                }
                break;
               
          
            // case StateL4Transition3:
            //     if(m_Wrist.getCurrentState() == WristStates.StateL4 && m_Pivot.getCurrentState() == PivotStates.StateL4){

            //         currentState=RobotStates.StateL4Finished;

            //     }
            //     break;
            case StateL4Finished:
                if(requestedState==RobotStates.StateCoralStationInit) {
                    currentState = RobotStates.StateL4RetractInit;
                }
                break;
            
            case StateL4RetractInit:
                // tuck gripper
                m_Wrist.requestState(WristStates.StateL4Tuck); // 25 rotor rotations
                currentState=RobotStates.StateL4RetractTransition1;
                // do some thing here :)
                break;
            case StateL4RetractTransition1:
                if(m_Wrist.getCurrentState() == WristStates.StateL4Tuck){
                    currentState =RobotStates.StateL4RetractInit2;

                }
                // blah
                break;
            case StateL4RetractInit2:
                 // elevator return to bottom
                 m_Elevator.requestState(ElevatorStates.StateStation); // then swing pivot
                currentState=RobotStates.StateL4RetractTransition2;

                //blah
                break;
            case StateL4RetractTransition2:
                if(m_Elevator.getCurrentState() == ElevatorStates.StateStation){
                    currentState =RobotStates.StateL4RetractInit3;

                }
                break;
            case StateL4RetractInit3:
                // pivot
                m_Pivot.requestState(PivotStates.StateL4Transition);
                currentState = RobotStates.StateL4Transition3;
                break;
            case StateL4Transition3:
                if(m_Pivot.getCurrentState() == PivotStates.StateL4Transition){ // 0.93
                    m_Wrist.requestState(WristStates.StateStation);

                }
                if(m_Pivot.getCurrentState() == PivotStates.StateL4Transition && m_Wrist.getCurrentState() == WristStates.StateStation){
                    m_Pivot.requestState(PivotStates.StateStation);
                    currentState=RobotStates.StateL4RetractFinal;

                } 
                break;
            case StateL4RetractFinal:
                currentState = RobotStates.StateCoralStationInit;
                break;
            case StateL2Algae:
                m_Elevator.requestState(ElevatorStates.StateL2Algae);
                if (m_Elevator.getCurrentState() == ElevatorStates.StateL2Algae){
                    m_Wrist.requestState(WristStates.StateL2Algae);
                    m_Pivot.requestState (PivotStates.StateL2Algae);
                }
                break;

            case StateL3Algae:
                m_Elevator.requestState(ElevatorStates.StateL3Algae);
                if (m_Elevator.getCurrentState() == ElevatorStates.StateL3Algae){
                    m_Wrist.requestState(WristStates.StateL3Algae);
                    m_Pivot.requestState (PivotStates.StateL3Algae);
                }
                break;
            

        }


        // if(getStateReached()){

        //     currentState=requestedState;

        // }
        // else{
        //     currentState = RobotStates.StateBetweenStates;

        // }
        state.setString(requestedState.toString());
    }
    public RobotStates getCurrentState(){
        return currentState;

    }
    public void requestRobotState(RobotStates changeTo){

        requestedState=changeTo;

    }

    // public boolean getStateReached(){

    //     if(m_Wrist.getRequestedState() == m_Wrist.getCurrentState() && m_Elevator.getRequestedState() == m_Elevator.getCurrentState() && m_Pivot.getCurrentState() == m_Pivot.getRequestedState()){
    //         return true;
    //     }
    //     else {
    //         return false;
    //     }

    // }
}
