package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Rotation;

import java.util.logging.Handler;

import com.ctre.phoenix6.signals.RobotEnableValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.StateHandler;
import frc.robot.StateHandler.RobotStates;
import frc.robot.commands.DriveCommandBuilder;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Vision.PoseEstimation;

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


public class AlignHandler extends SubsystemBase{
    public AlignStates currentState = AlignStates.StateInit;
    public AlignStates requestedState = AlignStates.StateDriving;
    private Drivetrain m_Drivetrain;
    private PoseEstimation m_PoseEstimation;
    private Command m_activeFollowCommand = null;
    private int currentTag = -1;
    private Command kteleOpCommand;
    private WL_CommandXboxController m_driver;
    private WL_CommandXboxController m_operator;
    StateHandler m_Handler;
    private double horizontalOffset;
    public enum AlignStates {

        StateInit,
        StateDriving,
        StateAlignRightInit,
        StateAlignLeftInit,
        StateRoughAlign,
        StateApproachInit,

        StateExtendL2Init,
        StateExtendL3Init,
        StateExtendL4Init,
        StateExtendL2,
        StateExtendL3,
        StateExtendL4,
        StateLower,
        StateRightApproachInit,
        StateLowerInit,
        StateLeftApproachInit,
        StateApproachRight,
        StateApproachLeft,
        StateApproach,
        StateAlignFinished,
        StateBackupInit,
        StateBackup
        
    }
    public AlignHandler(Drivetrain drivetrain, PoseEstimation poseEst, WL_CommandXboxController driver, StateHandler handler){ // include subsystems as argument
        m_driver = driver;
        m_Drivetrain = drivetrain;
        m_PoseEstimation = poseEst;
        m_Handler = handler;
        kteleOpCommand = new DriveWithController(
            m_driver::getLeftY,
            m_driver::getLeftX,
            m_driver::getRightX,
            () -> true,
            m_Drivetrain, m_PoseEstimation); 
    }
    
    GenericEntry state = Shuffleboard.getTab("Autos").add("alignerstate", "").getEntry();
    
    @Override
    public void periodic(){

        switch(currentState){

            case StateInit:
                currentState=requestedState;
                break;
            case StateDriving:
                CommandScheduler.getInstance().schedule(kteleOpCommand);

                if(requestedState == AlignStates.StateAlignRightInit || requestedState == AlignStates.StateAlignLeftInit ) currentState = requestedState;
                break; // put stuff for when the controllers are active;
            case StateAlignRightInit:
                horizontalOffset = .25;
                CommandScheduler.getInstance().cancel(kteleOpCommand);

                // temporarily removed below for testing;

                // int tagToTarget = 21; // replace with find nearest scorable tag logic

                m_activeFollowCommand = DriveCommandBuilder.roughAlignToTag(21, 1, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                currentState = AlignStates.StateRoughAlign;
                break;

            case StateAlignLeftInit:
                horizontalOffset = -.08;
                CommandScheduler.getInstance().cancel(kteleOpCommand);

                // temporarily removed below for testing;

                // int tagToTargetL = 21; // replace with find nearest scorable tag logic

                m_activeFollowCommand = DriveCommandBuilder.roughAlignToTag(21, 1, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                currentState = AlignStates.StateRoughAlign;
                break;
            case StateRoughAlign:
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()){
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                    m_activeFollowCommand = null;
                    currentState = AlignStates.StateExtendL2Init;


                }
                break;
            case StateExtendL2Init:
                m_Handler.requestRobotState(RobotStates.StateL2Init);
                currentState = AlignStates.StateExtendL2;

                break;
            case StateExtendL2:
                if(m_Handler.getCurrentState() == RobotStates.StateL2Finished){

                    currentState = AlignStates.StateRightApproachInit;

                }
                break;
            case StateRightApproachInit:
                // put in the command here that makes it go forward;
                Pose2d forwardPosRight = DriveCommandBuilder.convertAprilTag(21, 0.4, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight);
                
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
                currentState = AlignStates.StateApproach;
                break;

            case StateLeftApproachInit:
                // put in the command here that makes it go forward;
                Pose2d forwardPosLeft = DriveCommandBuilder.convertAprilTag(21, 0.4, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosLeft);
                
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
                currentState = AlignStates.StateApproach;
                break;

            // case StateLeftApproachInit:
            //     // put in the command here that makes it go forward;
            //     Pose2d forwardPos = DriveCommandBuilder.convertAprilTag(21, 0.4, .25, m_Drivetrain, m_PoseEstimation);
            //     m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPos);
                
            //     CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
            //     currentState = AlignStates.StateApproach;
            //     break;
            case StateApproach:
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()){
                    currentState = AlignStates.StateBackupInit;
                    m_activeFollowCommand=null;
                }
                break;    
            case StateBackupInit:
                Pose2d backPos = DriveCommandBuilder.convertAprilTag(21, 1, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, backPos);
                
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
            
                currentState = AlignStates.StateBackup;
                break;
            case StateBackup:
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()){
                    currentState = AlignStates.StateLowerInit;
                    m_activeFollowCommand=null;
                }
                break; 
            case StateLowerInit:
                m_Handler.requestRobotState(RobotStates.StateCoralStationInit);
                currentState = AlignStates.StateLower;
                break;
            case StateLower:
                if (m_Handler.getCurrentState() == RobotStates.StateCoralStationFinal){
                    currentState = AlignStates.StateAlignFinished;
                }
                break;
                
            case StateAlignFinished:
                requestedState=AlignStates.StateDriving;
                currentState = requestedState; // get out of this state!
                break;
        }


        // if(getStateReached()){

        //     currentState=requestedState;

        // }
        // else{
        //     currentState = RobotStates.StateBetweenStates;

        // }
        state.setString(currentState.toString());
    }
    public void requestAlignState(AlignStates changeTo){

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
