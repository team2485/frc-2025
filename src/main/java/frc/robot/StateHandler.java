package frc.robot;

import static frc.robot.Constants.WristConstants.*;

import java.net.Authenticator.RequestorType;
import java.rmi.server.RemoteObjectInvocationHandler;
import java.security.cert.PKIXReason;
import java.time.chrono.ChronoPeriod;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.RequiresUserReason;

import static frc.robot.Constants.ElevatorConstants.*;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.SynchronousInterrupt.WaitResult;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberStates;
import frc.robot.subsystems.PieceHandling.Elevator;
import frc.robot.subsystems.PieceHandling.Pivot;
import frc.robot.subsystems.PieceHandling.Wrist;
import frc.robot.subsystems.PieceHandling.Elevator.ElevatorStates;
import frc.robot.subsystems.PieceHandling.Pivot.PivotStates;
import frc.robot.subsystems.PieceHandling.Wrist.WristStates;
import frc.robot.subsystems.drive.AlignHandler;
import frc.robot.subsystems.drive.AlignHandler.AlignStates;

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
    private Climber m_Climber;
    private AlignHandler m_aligner; 
    private RobotContainer m_container;
    public enum RobotStates {

        StateBetweenStates,
        StateInit,
        StateZero,
        StateCoralStationInit,
        StateCoralStationTransition,
        StateCoralStationTransition2,
        StateCoralStationFinal,
        StateL1Init,
        StateL1Final,
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
        StateL2AlgaeFinal,
        StateL2AlgaeTransition,
        StateProcessorFinal,
        StateL2AlgaeInit,
        StateL3AlgaeFinal,
        StateL3AlgaeTransition,
        StateL3AlgaeInit,
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
        StateBargeInit,
        StateBargeFinal,
        StateL4RetractInit,
        StateL2Prepare,
        StateL4Prepare1,
        StateL4Prepare2,
        StateL3Prepare,
        StateL4Prepare,
        StateAbort,
        StateL2Algae,
        StateClimbPause,
        StateL3Algae,
        StateLollipopInit,
        StateLollipopFinal,
        StateProcessorInit,
        StateClimberPrepare,
        StateClimbGo
    }

    public StateHandler(Elevator elevator, Wrist wrist, Pivot pivot,Climber climber, RobotContainer cont ){ // include subsystems as argument
        m_Climber=climber;
        m_container = cont;
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
                m_Climber.requestState(ClimberStates.StateClimberOff);
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
            case StateL1Init:
                m_Wrist.requestState(WristStates.StateL1);
                m_Pivot.requestState(PivotStates.StateL1);
                currentState = RobotStates.StateL1Final;
                break;

            case StateL1Final:
                if(requestedState == RobotStates.StateCoralStationInit){

                    currentState=requestedState;

                }
                break;

            case StateL2Prepare:
                m_Wrist.requestState(WristStates.StateL2);
                m_Pivot.requestState(PivotStates.StateL2);
                if (requestedState == RobotStates.StateCoralStationInit || requestedState == RobotStates.StateL2Init){
                    currentState = requestedState;
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
                if(requestedState==RobotStates.StateCoralStationInit ) {
                    currentState = RobotStates.StateCoralStationInit;
                }
                break;
            case StateL3Prepare:
                m_Wrist.requestState(WristStates.StateL3);
                m_Pivot.requestState(PivotStates.StateL3);
                if (requestedState == RobotStates.StateCoralStationInit || requestedState == RobotStates.StateL3Init){
                    currentState = requestedState;
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
            case StateClimberPrepare:
                if( requestedState==RobotStates.StateCoralStationInit || requestedState==RobotStates.StateClimbGo  ||requestedState==RobotStates.StateClimbPause  ){

                    currentState=requestedState;

                }
                if(m_Pivot.getCurrentState() == PivotStates.StateClimb){

                    m_Wrist.requestState(WristStates.StateClimb);


                }
                m_Climber.requestState(ClimberStates.StateClimberOnForward);
                m_Pivot.requestState(PivotStates.StateClimb);
                m_Elevator.requestState(ElevatorStates.StateStation);
                break;
            case StateClimbGo:
                if( requestedState==RobotStates.StateClimberPrepare ||requestedState==RobotStates.StateClimbPause ){

                    currentState=requestedState;

                }
                m_Climber.requestState(ClimberStates.StateClimberOnBackward);
                m_Pivot.requestState(PivotStates.StateClimb);
                m_Elevator.requestState(ElevatorStates.StateStation);
                if(m_Pivot.getCurrentState() == PivotStates.StateClimb){

                    m_Wrist.requestState(WristStates.StateClimb);


                }
                break;
            case StateClimbPause:

                if( requestedState==RobotStates.StateClimberPrepare || requestedState==RobotStates.StateClimbGo || requestedState == RobotStates.StateCoralStationInit ){

                    currentState=requestedState;

                }
                m_Climber.requestState(ClimberStates.StateClimberOff);
                m_Pivot.requestState(PivotStates.StateClimb);
                m_Elevator.requestState(ElevatorStates.StateStation);
                if(m_Pivot.getCurrentState() == PivotStates.StateClimb){

                    m_Wrist.requestState(WristStates.StateClimb);


                }
                break;

            case StateL4Prepare1:
                m_Pivot.requestState(PivotStates.StateL4);
                m_Elevator.requestState(ElevatorStates.StateL4Half);
                currentState = RobotStates.StateL4Prepare2;
                break;
            case StateL4Prepare2:
                if(m_Elevator.getCurrentState() == ElevatorStates.StateL4Half){
                    if(requestedState==RobotStates.StateL4Init || requestedState==RobotStates.StateCoralStationInit){

                        currentState=requestedState;

                    }
                    m_Wrist.requestState(WristStates.StateL4);

                }
                if(requestedState==RobotStates.StateL4Init || requestedState==RobotStates.StateCoralStationInit){

                    currentState=requestedState;

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
                    currentState = RobotStates.StateCoralStationInit;
                }
                if(requestedState == RobotStates.StateL3AlgaeInit && m_container.m_Aligner.getCurrentState() == AlignStates.StateLower){

                    currentState = RobotStates.StateL3AlgaeInit;

                }
                if(requestedState == RobotStates.StateL2AlgaeInit && m_container.m_Aligner.getCurrentState() == AlignStates.StateLower){

                    currentState = RobotStates.StateL2AlgaeInit;

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
            case StateL2AlgaeInit:
                m_Elevator.requestState(ElevatorStates.StateL2Algae);
                currentState = RobotStates.StateL2AlgaeTransition;
                break;
            case StateL3AlgaeInit:
                m_Elevator.requestState(ElevatorStates.StateL3Algae);
                currentState = RobotStates.StateL3AlgaeTransition;
                break;
            case StateL2AlgaeTransition:
                if (m_Elevator.getCurrentState() == ElevatorStates.StateL2Algae){
                    m_Wrist.requestState(WristStates.StateL2Algae);
                    m_Pivot.requestState (PivotStates.StateL2Algae);
                    currentState =  RobotStates.StateL2AlgaeFinal;
                }
                break;
            case StateL3AlgaeTransition:
                if (m_Elevator.getCurrentState() == ElevatorStates.StateL3Algae){
                    m_Wrist.requestState(WristStates.StateL3Algae);
                    m_Pivot.requestState (PivotStates.StateL3Algae);
                    currentState =  RobotStates.StateL3AlgaeFinal;
                }
                break;
            case StateL2AlgaeFinal:
                if(requestedState==RobotStates.StateCoralStationInit) {
                    currentState = RobotStates.StateCoralStationInit;
                }
                break;
                

            case StateL3AlgaeFinal:
                if(requestedState==RobotStates.StateCoralStationInit) {
                    currentState = RobotStates.StateCoralStationInit;
                }
                break;
            
            case StateProcessorInit:
                m_Elevator.requestState(ElevatorStates.StateProcessor);
                m_Wrist.requestState(WristStates.StateProcessor);
                currentState = RobotStates.StateProcessorFinal;
                break;
            case StateProcessorFinal:
                if(requestedState == RobotStates.StateCoralStationInit){

                    currentState=requestedState;

                }
            break;

            case StateBargeInit:
                m_Elevator.requestState(ElevatorStates.StateBarge);
                m_Wrist.requestState(WristStates.StateBarge);
                m_Pivot.requestState(PivotStates.StateBarge);
                currentState = RobotStates.StateBargeFinal;
                break;
            
            case StateBargeFinal:
                if(requestedState == RobotStates.StateCoralStationInit){

                    currentState=requestedState;

                }
            break;

            case StateLollipopInit:
                m_Wrist.requestState(WristStates.StateLollipop);
                m_Pivot.requestState(PivotStates.StateLollipop);
                currentState = RobotStates.StateLollipopFinal;
                break;

            case StateLollipopFinal:
                if(requestedState == RobotStates.StateCoralStationInit){

                    currentState=requestedState;

                }
                break;

            // case StateBargeInit:
            //     m_Elevator.requestState(ElevatorStates.StateBarge);
            //     m_Wrist.requestState(WristStates.StateBarge);
            //     m_Pivot.requestState(PivotStates.StateBarge);
            //     currentState = RobotStates.StateBargeFinal;
            //     break;
            
            // case StateBargeFinal:
            //     if(requestedState == RobotStates.StateCoralStationInit){

            //         currentState=requestedState;

            //     }
            // break;
            
            // case StateBargeInit:
            //     m_Elevator.requestState(ElevatorStates.StateBarge);
            //     currentState = RobotStates.StateBargeTransition;
            //     break;
            // case StateBargeTransition:
            //     if(m_Elevator.getCurrentState() == ElevatorStates.StateBarge){

            //         m_Wrist.requestState(WristStates.StateBarge);
            //         m_Pivot.requestState(PivotStates.StateBarge);
            //         currentState= RobotStates.StateBargeTransition2;
            //     }
            //     break;  
            // case StateBargeTransition2:
            //     if(m_Wrist.getCurrentState() == WristStates.StateBarge && m_Pivot.getCurrentState() == PivotStates.StateBarge){

            //         currentState = RobotStates.StateBargeFinal;

            //     }
            //     break;
            // case StateBargeFinal:
            //     if(requestedState == RobotStates.StateCoralStationInit){

            //         currentState = RobotStates.StateL4RetractInit;

            //     }
            //     break;
           
            case StateAbort:
                if(currentState == RobotStates.StateL4Finished ){

                    currentState = RobotStates.StateL4RetractInit;
                    requestedState = RobotStates.StateCoralStationInit;

                }else{
                    currentState = RobotStates.StateCoralStationInit;
                    requestedState=RobotStates.StateCoralStationInit;
                    

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
        if(changeTo == RobotStates.StateAbort)
            currentState=RobotStates.StateAbort;
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
