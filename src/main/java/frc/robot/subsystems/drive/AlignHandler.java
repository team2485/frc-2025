package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.StateHandler;
import frc.robot.StateHandler.RobotStates;
import frc.robot.commands.DriveCommandBuilder;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PieceHandling.Roller;
import frc.robot.subsystems.PieceHandling.Roller.RollerStates;
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
    private Roller m_roller;
    private AlignStates desiredExtension=AlignStates.StateInit;
    private LED m_leds;
    private RobotContainer m_Container;
    public enum AlignStates {

        StateInit,
        StateAuto,
        StateCoralStationExtension,
        StateDriving,
        StateAlignRightInit,
        StateAlignLeftInit,
        StateAlignRightL2Init,
        StateAlignLeftL2Init,
        StateAlignRightL4Init,
        StateAlignLeftL4Init,
        StateRoughAlign,
        StateApproachInit,
        StateAbort,
        StateAlignMidInit,
        StateAlignAlgaeL2Init,
        StateExtendL2Init,
        StateExtendL2AlgaeInit,
        StateAlignAlgaeL3Init,
        StateExtendL3AlgaeInit,
        StateExtendL3Init,
        StateExtendL4Init,
        StateExtendL2,
        StateExtendL2Algae,
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
        StateBackup,
        StateExtendL3Algae,
        StateCoralFinished,
        StateCoralApproachInit,

        StateAlignLeftL3Init,
        StateCoralStationInit,
        StateAlignRightL3Init, StateCoralStationRoughAlign, StateAlignCoralStationInit, StateCoralApproach, StateAutoBackupInit
        
    }


    private double speedLimit = 1.0;

    public void requestSpeedLimit(double speedLimit){
        this.speedLimit = speedLimit;
    }

    private double speedMult = 1.0;
    public double getSpeedMultiplier(){

        return speedMult;

    }
    public AlignStates getCurrentState(){

        return currentState;

    }
    public boolean isAllowedToDrive(){

        if(currentState == AlignStates.StateLower || currentState == AlignStates.StateLowerInit || currentState == AlignStates.StateDriving || currentState == AlignStates.StateAuto || currentState == AlignStates.StateAlignFinished || currentState == AlignStates.StateCoralFinished || currentState == AlignStates.StateAutoBackupInit){

            return true;

        }
        return false;
    }
    int count = 0;

    public AlignHandler(Drivetrain drivetrain, PoseEstimation poseEst, WL_CommandXboxController driver, StateHandler handler, Roller rollers, RobotContainer cont){ // include subsystems as argument
        m_driver = driver;
        m_Container = cont;
        m_Drivetrain = drivetrain;
        m_PoseEstimation = poseEst;
        m_Handler = handler;
        m_roller=rollers;
        m_leds = new LED();
        kteleOpCommand = new DriveWithController(
            m_driver::getLeftY,
            m_driver::getLeftX,
            m_driver::getRightX,
            this::getSpeedMultiplier,
            () -> true,
            m_Drivetrain, m_PoseEstimation);
    }
    
    GenericEntry state = Shuffleboard.getTab("Autos").add("alignerstate", "").getEntry();
    GenericEntry tagLog = Shuffleboard.getTab("Autos").add("tag log", -1).getEntry();
    GenericEntry horizOfLog = Shuffleboard.getTab("Autos").add("horizontal offset", -1).getEntry();
    long stateBackupStartTime = -1;

    public void forceState(AlignStates state){ // use only for tele-op transitions or aborts

        requestedState = state;
        currentState=state;

    }

    @Override
    public void periodic(){
        int targetID = -1;
        double coralSign=1;
        if(m_driver.b().getAsBoolean()){

            currentState=AlignStates.StateAbort;

        }
        switch(currentState){

            case StateInit:
                currentState=requestedState;
                break;
            case StateAuto:
                currentState=requestedState;
                break;
            case StateDriving:
                // speedMult = 1;
                if(speedMult < speedLimit){
                    speedMult+=0.003;

                }
                else{
                    speedMult = speedLimit;

                }
                if(m_activeFollowCommand != null){

                    m_activeFollowCommand.cancel();
                    m_activeFollowCommand = null;
                    //m_Drivetrain.drive(Translation2d.kZero, 0, isAllowedToDrive(), isAllowedToDrive(), null);
                    m_Drivetrain.driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0,m_PoseEstimation.getCurrentPose().getRotation()));
                }
                
                CommandScheduler.getInstance().schedule(kteleOpCommand); 


                //whitelist 
                if(requestedState == AlignStates.StateAlignRightL2Init || requestedState == AlignStates.StateAlignLeftL2Init || 
                requestedState == AlignStates.StateAlignRightL4Init || 
                requestedState == AlignStates.StateAlignLeftL4Init 
                || requestedState == AlignStates.StateAlignAlgaeL2Init || 
                requestedState == AlignStates.StateAlignLeftL3Init ||
                requestedState == AlignStates.StateAlignRightL3Init ||
                requestedState==AlignStates.StateAlignAlgaeL3Init ||
                requestedState == AlignStates.StateAuto
                
                ) currentState = requestedState;

                break; // put stuff for when the controllers are active;
            case StateAlignLeftL2Init:
                desiredExtension=AlignStates.StateExtendL2Init;
                currentState=AlignStates.StateAlignLeftInit;
                break;
            case StateAlignLeftL4Init:

                desiredExtension=AlignStates.StateExtendL4Init;
                currentState=AlignStates.StateAlignLeftInit;
                break;
            case StateAlignRightL2Init:

                desiredExtension=AlignStates.StateExtendL2Init;
                currentState=AlignStates.StateAlignRightInit;
                break;
            case StateAlignRightL3Init:

                desiredExtension=AlignStates.StateExtendL3Init;
                currentState=AlignStates.StateAlignRightInit;
                break;
            case StateAlignLeftL3Init:

                desiredExtension=AlignStates.StateExtendL3Init;
                currentState=AlignStates.StateAlignLeftInit;
                break;
            case StateAlignRightL4Init:

                desiredExtension=AlignStates.StateExtendL4Init;
                currentState=AlignStates.StateAlignRightInit;
                break;
            case StateAlignAlgaeL2Init:
                desiredExtension = AlignStates.StateExtendL2AlgaeInit;
                currentState = AlignStates.StateAlignMidInit;
                break;
            case StateAlignAlgaeL3Init:
                desiredExtension = AlignStates.StateExtendL3AlgaeInit;
                currentState = AlignStates.StateAlignMidInit;
                break;

            case StateAlignCoralStationInit:
                //desiredExtension = AlignStates.StateCoralStationInit;
                if(!DriverStation.isAutonomousEnabled())
                {
                    
                    
                    m_Handler.requestRobotState(RobotStates.StateCoralStationInit);
                }
                    targetID = DriveCommandBuilder.findNearestSourceId(m_PoseEstimation,m_Drivetrain);
               // horizontalOffset = .255;
                CommandScheduler.getInstance().cancel(kteleOpCommand);
                if(m_activeFollowCommand!=null)
                m_activeFollowCommand.cancel();
                // temporarily removed below for testing;

                // int tagToTarget = 21;appr // replace with find nearest scorable tag logic
                // Pose2d coralStationTargetPose = DriveCommandBuilder.convertAprilTag(targetID, 1.3, 0, m_Drivetrain, m_PoseEstimation);
                // m_activeFollowCommand =DriveCommandBuilder.shortDriveToPoseFast(m_Drivetrain, m_PoseEstimation, coralStationTargetPose);// DriveCommandBuilder.roughAlignToTag(targetID, 1.3, 0, m_Drivetrain, m_PoseEstimation,true);
                // CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                currentState = AlignStates.StateCoralStationRoughAlign;

                break;
            case StateCoralStationRoughAlign:
            //    if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()){
                    // CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                    // m_activeFollowCommand = null;
                    
                    currentState = AlignStates.StateCoralApproachInit;
                // }
                break;
            case StateCoralApproachInit:
                targetID = DriveCommandBuilder.findNearestSourceId(m_PoseEstimation,m_Drivetrain);

                // put in the command here that makes it go forward;
                 coralSign = 1;
                if(targetID == 13){ // TODO: ADD OTHER CORAL STATION ID FOR RED 
                    coralSign=-1;


                }
                Pose2d forwardPosCoral = DriveCommandBuilder.convertAprilTag(targetID, 0.3, -.5*coralSign, m_Drivetrain, m_PoseEstimation,true);
                

                m_activeFollowCommand = DriveCommandBuilder.shortDriveToCoralStation(m_Drivetrain, m_PoseEstimation, forwardPosCoral);
                
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
                currentState = AlignStates.StateCoralApproach;
                break;
            case StateCoralApproach:
            targetID = DriveCommandBuilder.findNearestSourceId(m_PoseEstimation,m_Drivetrain);
                
            Pose2d distCheckforwardPosCoral =DriveCommandBuilder.convertAprilTag(targetID, 0.4, -.5*coralSign, m_Drivetrain, m_PoseEstimation,true);
                double dist = distCheckforwardPosCoral.getTranslation().getDistance(m_PoseEstimation.getCurrentPose().getTranslation()); 
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()  ){//&& m_activeFollowCommand != null){
                    m_activeFollowCommand.cancel();
                   // CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                    m_Drivetrain.driveAuto(new ChassisSpeeds(-0.5, 0,0));
                    m_activeFollowCommand = null;
                    
                    currentState = AlignStates.StateCoralFinished;
                }
                break;
            case StateCoralFinished:
                currentState = AlignStates.StateAlignFinished;
                requestedState=AlignStates.StateAuto;
                break;
            case StateAlignRightInit:

                
                targetID = DriveCommandBuilder.findNearestScoringTagId(m_PoseEstimation);

                horizontalOffset = .29;
                if(DriverStation.isAutonomous()){
                    horizontalOffset=.27;

                }

                if(PoseEstimation.getFieldConstants().isOnRed()){

                    int index = targetID - 6;
   
                    double addedOffset = PoseEstimation.getFieldConstants().getReefXOffsetsRight()[index]; 
                    
                    
                    horizontalOffset+=addedOffset;

                }
                else{

                    int index = targetID - 17;
                    double addedOffset = PoseEstimation.getFieldConstants().getReefXOffsetsRight()[index]; 
                    horizontalOffset+=addedOffset;

                }
                // if(DriverStation.getAlliance().equals(Alliance.Blue) ){

                //     int index = targetID - 17;
                //     double addedOffset = m_PoseEstimation.getFieldConstants().getReefXOffsetsRight()[index]; 
                //     horizontalOffset+=addedOffset;

                // }
                // if(DriverStation.getAlliance().equals(Alliance.Red) ){

                    

                // }

                CommandScheduler.getInstance().cancel(kteleOpCommand);

                // temporarily removed below for testing;

                // int tagToTarget = 21; // replace with find nearest scorable tag logic
                if(DriverStation.isAutonomous()){
                    m_activeFollowCommand = DriveCommandBuilder.roughAlignToTag(targetID, 1.3, horizontalOffset, m_Drivetrain, m_PoseEstimation);


                }
                else{
                    m_activeFollowCommand = DriveCommandBuilder.roughAlignToTag(targetID, .85, horizontalOffset, m_Drivetrain, m_PoseEstimation);


                }
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                currentState = AlignStates.StateRoughAlign;
                if(desiredExtension==AlignStates.StateExtendL2Init){

                    m_Handler.requestRobotState(RobotStates.StateL2Init);
                    //currentState = AlignStates.StateExtendL2;

                }
                break;

            case StateAlignLeftInit:
                horizontalOffset = -0.03;
                if(DriverStation.isAutonomous()){
                    horizontalOffset=-0.05;

                }
                targetID = DriveCommandBuilder.findNearestScoringTagId(m_PoseEstimation);


                // if(DriverStation.getAlliance().equals(Alliance.Blue) ){

                //     int index = targetID - 17;
                //     double addedOffset = m_PoseEstimation.getFieldConstants().getReefXOffsetsLeft()[index]; 
                //     horizontalOffset+=addedOffset;

                // }
                //if(DriverStation.getAlliance().equals(Alliance.Red) ){

                if(m_PoseEstimation.getFieldConstants().isOnRed()){

                    int index = targetID - 6;
                    double addedOffset = m_PoseEstimation.getFieldConstants().getReefXOffsetsLeft()[index]; 
                    horizontalOffset+=addedOffset;

                }
                else{

                    int index = targetID - 17;
                    double addedOffset = m_PoseEstimation.getFieldConstants().getReefXOffsetsLeft()[index]; 
                    horizontalOffset+=addedOffset;

                }
                //}


                CommandScheduler.getInstance().cancel(kteleOpCommand);

                // temporarily removed below for testing;

                // int tagToTargetL = 21; // replace with find nearest scorable tag logic

                m_activeFollowCommand = DriveCommandBuilder.roughAlignToTag(targetID, .85, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                currentState = AlignStates.StateRoughAlign; 
                
                if(desiredExtension==AlignStates.StateExtendL2Init){

                    m_Handler.requestRobotState(RobotStates.StateL2Init);
                    //currentState = AlignStates.StateExtendL2;

                }

                break;
            case StateAlignMidInit:
                horizontalOffset = 0.1
                ;
                targetID = DriveCommandBuilder.findNearestScoringTagId(m_PoseEstimation);
                CommandScheduler.getInstance().cancel(kteleOpCommand);

                // temporarily removed below for testing;

                // int tagToTargetL = 21; // replace with find nearest scorable tag logic

                m_activeFollowCommand = DriveCommandBuilder.roughAlignToTag(targetID, .85, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                currentState = AlignStates.StateRoughAlign; 
                
                if(desiredExtension==AlignStates.StateExtendL2AlgaeInit){

                    m_Handler.requestRobotState(RobotStates.StateL2AlgaeInit);
                    //currentState = AlignStates.StateExtendL2;

                }

                break;
            case StateRoughAlign:
                // if(DriverStation.isAutonomousEnabled()){

                //     if(m_activeFollowCommand != null  && desiredExtension != AlignStates.StateExtendL2Init && desiredExtension != AlignStates.StateExtendL2AlgaeInit){
                //         CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                //         m_activeFollowCommand = null;
                        
                //         currentState = desiredExtension;
    
    
                //     }

                // }
                m_leds.requestState(LED.LEDStates.StateYellow);
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished() && desiredExtension != AlignStates.StateExtendL2Init && desiredExtension != AlignStates.StateExtendL2AlgaeInit){
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                    m_activeFollowCommand = null;
                    
                    currentState = desiredExtension;


                } else if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished() && desiredExtension == AlignStates.StateExtendL2Init){
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                    m_activeFollowCommand = null;
                    currentState = AlignStates.StateExtendL2;
                }
                else if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished() && desiredExtension == AlignStates.StateExtendL2AlgaeInit){
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                    m_activeFollowCommand = null;
                    currentState = AlignStates.StateExtendL2Algae;
                }

                break;
            case StateExtendL2Algae:
                
                // if(m_Handler.getCurrentState() == RobotStates.StateL2AlgaeFinal){

                    currentState = AlignStates.StateApproachInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                    m_activeFollowCommand = null;
                // }
                break;
            case StateExtendL3AlgaeInit:
                CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                m_Handler.requestRobotState(RobotStates.StateL3AlgaeInit);
                currentState = AlignStates.StateExtendL3Algae;
                break;
            case StateExtendL2Init:
            CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                m_Handler.requestRobotState(RobotStates.StateL2Prepare);
                currentState = AlignStates.StateExtendL2;

                break;
            case StateExtendL3Init:
                CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                m_Handler.requestRobotState(RobotStates.StateL3Init);
                currentState = AlignStates.StateExtendL3;
                break;
            case StateExtendL3Algae:
                // if(m_Handler.getCurrentState() == RobotStates.StateL3AlgaeFinal){

                    currentState = AlignStates.StateApproachInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                    m_activeFollowCommand = null;

                // }
                break;
            case StateExtendL3:
        
                // if(m_Handler.getCurrentState() == RobotStates.StateL3Finished){

                    currentState = AlignStates.StateApproachInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                    m_activeFollowCommand = null;
                // }
                break;
            case StateExtendL4Init:
                m_Handler.requestRobotState(RobotStates.StateL4Init);
                CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                currentState = AlignStates.StateExtendL4;
                break;
            case StateExtendL4:
                if(DriverStation.isAutonomousEnabled()){

                    currentState = AlignStates.StateApproachInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                    m_activeFollowCommand = null;

                }
                // if(m_Handler.getCurrentState() == RobotStates.StateL4Finished ){//|| m_Handler.getCurrentState()==RobotStates.StateCoralStationFinal || m_Handler.getCurrentState() == RobotStates.StateCoralStationInit){

                    currentState = AlignStates.StateApproachInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                    m_activeFollowCommand = null;
                // }
                break;
            case StateExtendL2:
                // if(m_Handler.getCurrentState() == RobotStates.StateL2Finished){

                    currentState = AlignStates.StateApproachInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                   m_activeFollowCommand = null;
                // }
                break;
            case StateApproachInit:

                double forwardOffsetApproach = 0.565; // yuvi code says .43 but need to account for bend in wrist
                
                if(DriverStation.isAutonomous()){

                    forwardOffsetApproach=.54;

                }
                if (desiredExtension == AlignStates.StateExtendL2Init || desiredExtension == AlignStates.StateExtendL3Init){
                    forwardOffsetApproach = 0.4;
                }
                if(desiredExtension == AlignStates.StateExtendL2AlgaeInit || desiredExtension == AlignStates.StateExtendL3AlgaeInit){
                    m_roller.requestState(RollerStates.StateAlgaeIntake);


                }
                targetID = DriveCommandBuilder.findNearestScoringTagId(m_PoseEstimation);

                // put in the command here that makes it go forward;

                Pose2d forwardPosRight = DriveCommandBuilder.convertAprilTag(targetID, forwardOffsetApproach, horizontalOffset, m_Drivetrain, m_PoseEstimation);

                
                
                if(!DriverStation.isAutonomousEnabled()){

                    PathConstraints constraints = new PathConstraints(4,3.2, 1.5,1);
                    m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
             
                // m_activeFollowCommand = DriveCommandBuilder.shortDriveToPoseMid(m_Drivetrain, m_PoseEstimation, forwardPosRight);

                }
                else{
                    PathConstraints constraints = new PathConstraints(1.5, 0.75, 0.5, 0.5);//new PathConstraints(1, 1, 0.5,0.5);
                    m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
                    
                }   
                
                if (desiredExtension != AlignStates.StateExtendL4Init){
                    PathConstraints constraints = new PathConstraints(5, 4.5, 1, 1);//new PathConstraints(1, 1, 0.5,0.5);
                    m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
                }

                if (desiredExtension == AlignStates.StateExtendL2Init){
                    PathConstraints constraints = new PathConstraints(6, 4.5, 1, 1);//new PathConstraints(1, 1, 0.5,0.5);
                    m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
                }
                if(DriverStation.isAutonomous()){
                    
                    PathConstraints constraints = new PathConstraints(3, 1.7,1, 1, 12);//new PathConstraints(1, 1, 0.5,0.5);
                   
                    Pose2d roughAlignPos = DriveCommandBuilder.convertAprilTag(targetID, 1.2, horizontalOffset,m_Drivetrain,m_Container.m_poseEstimation);
                
                    // if(count == 0){

                    //     constraints=new PathConstraints(3, 3,0.75, 0.75, 12);

                    // }

                    


                    m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose3Waypoint(m_Drivetrain, m_PoseEstimation, roughAlignPos, forwardPosRight, constraints);
                    count++;


                }


                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
                currentState = AlignStates.StateApproach;
                horizOfLog.setDouble(horizontalOffset);

                break;


                // if(desiredExtension == AlignStates.StateExtendL2AlgaeInit || desiredExtension == AlignStates.StateExtendL3AlgaeInit){
                //     m_roller.requestState(RollerStates.StateAlgaeIntake);


                // }

                // if (desiredExtension == AlignStates.StateExtendL2 || desiredExtension == AlignStates.StateExtendL3){
                //     forwardOffsetApproach = 0.29;
                // }
                // targetID = DriveCommandBuilder.findNearestScoringTagId(m_PoseEstimation);

                // // put in the command here that makes it go forward;
                
                // if(DriverStation.isAutonomous()){

                //     forwardOffsetApproach = 0.51;

                // }
                // Pose2d forwardPosRight = DriveCommandBuilder.convertAprilTag(targetID, forwardOffsetApproach,horizontalOffset, m_Drivetrain, m_PoseEstimation);
                
                
                // if(!DriverStation.isAutonomousEnabled()){

                //     PathConstraints constraints = new PathConstraints(2, 2, 0.5,0.5);

    
                //     m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
                //     if (desiredExtension != AlignStates.StateExtendL4Init){
                //         constraints = new PathConstraints(5, 4, 0.5, 0.5);//new PathConstraints(1, 1, 0.5,0.5);
                //         m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
                //     }
                // }
                // else{
                //     PathConstraints constraints = new PathConstraints(2.5, 2, 0.5, 0.5);//new PathConstraints(1, 1, 0.5,0.5);
                //     m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPosRight, constraints);
                    
                // }   
                

                // CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
                // currentState = AlignStates.StateApproach;
                // horizOfLog.setDouble(horizontalOffset);

                // break;

            // case StateLeftApproachInit:
            //     // put in the command here that makes it go forward;
            //     Pose2d forwardPos = DriveCommandBuilder.convertAprilTag(21, 0.4, .25, m_Drivetrain, m_PoseEstimation);
            //     m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, forwardPos);
                
            //     //CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                
            //     currentState = AlignStates.StateApproach;
            //     break;
            case StateApproach:
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()){
                    if(DriverStation.isAutonomousEnabled()){



                        currentState=AlignStates.StateAutoBackupInit;
                    }
                    else{
                    currentState = AlignStates.StateBackupInit;

                    }
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                    m_activeFollowCommand=null;
                    if(desiredExtension != AlignStates.StateExtendL2AlgaeInit && desiredExtension != AlignStates.StateExtendL3AlgaeInit){
                        m_roller.requestState(RollerStates.StateRollerOnBackward);


                    }
                    
                }
                break;  
            case StateAutoBackupInit:
                
                if(requestedState==AlignStates.StateAlignCoralStationInit){
                    currentState = requestedState   ;

                }
                break;
            case StateBackupInit: // .7 meters should be ok
                double forwardOffset = 1;
  
                if(desiredExtension == AlignStates.StateExtendL2AlgaeInit || desiredExtension == AlignStates.StateExtendL3AlgaeInit || m_Container.getDesiredLevel() == 6){
                    forwardOffset=1.5;

                }
                targetID = DriveCommandBuilder.findNearestScoringTagId(m_PoseEstimation);

                Pose2d backPos = DriveCommandBuilder.convertAprilTag(targetID, forwardOffset, horizontalOffset, m_Drivetrain, m_PoseEstimation);
                PathConstraints constraints = new PathConstraints(7, 5, 1.3, 1.3);
                m_activeFollowCommand = DriveCommandBuilder.shortDriveToPose(m_Drivetrain, m_PoseEstimation, backPos,constraints);
                
                CommandScheduler.getInstance().schedule(m_activeFollowCommand);
            
                currentState = AlignStates.StateBackup;
                stateBackupStartTime = System.currentTimeMillis();
                break;
            case StateBackup:
                // if(DriverStation.isAutonomousEnabled()){

                //     currentState = AlignStates.StateLowerInit;
                //     CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                //     m_activeFollowCommand=null;

                // }
                // if(System.currentTimeMillis() - stateBackupStartTime > 1500){
                //     currentState = AlignStates.StateBackupInit;

                //     CommandScheduler.getInstance().cancel(m_activeFollowCommand);


                // }
                if(m_activeFollowCommand != null && m_activeFollowCommand.isFinished()){
                    currentState = AlignStates.StateLowerInit;
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);

                   m_activeFollowCommand=null;
                   
                }
                break; 
            case StateLowerInit:
                speedMult = 0.2;
                if(desiredExtension != AlignStates.StateExtendL2AlgaeInit && desiredExtension != AlignStates.StateExtendL3AlgaeInit){
                    if(!DriverStation.isAutonomousEnabled())
                    m_roller.requestState(RollerStates.StateRollerOff);


                }
       
                
                //if(m_Handler.getRequestedState()
                // if(m_Container.getDesiredLevel() == 6){

                //     m_Handler.requestRobotState(RobotStates.StateL3AlgaeInit);


                // }else if(m_Container.getDesiredLevel() == 5){
                //     m_Handler.requestRobotState(RobotStates.StateL2AlgaeInit);

                // }
                // else{
                if(m_Container.getDesiredLevel()!=6 && m_Container.getDesiredLevel()!=5){

                    m_Handler.requestRobotState(RobotStates.StateCoralStationInit);


                }
                else{
                    m_Handler.requestRobotState(RobotStates.StateProcessorInit);


                }


                currentState = AlignStates.StateLower;
                break;
            case StateLower:
                if(!DriverStation.isAutonomous()){

                    CommandScheduler.getInstance().schedule(kteleOpCommand);
                    

                }
                else{
                    if(requestedState != AlignStates.StateAlignCoralStationInit ){

                        requestedState=AlignStates.StateAlignFinished;


                    } else{
                        currentState = requestedState;
                    }
                    
                }
                if(speedMult < speedLimit){
                    speedMult+=0.003;

                }
                else{
                    speedMult = speedLimit;

                }

                if (m_Handler.getCurrentState() == RobotStates.StateCoralStationFinal || m_Handler.getCurrentState()==RobotStates.StateL3AlgaeFinal || m_Handler.getCurrentState() == RobotStates.StateL2AlgaeFinal){
                    currentState = AlignStates.StateAlignFinished;
                    if(desiredExtension != AlignStates.StateExtendL2AlgaeInit && desiredExtension != AlignStates.StateExtendL3AlgaeInit){
                        if(!DriverStation.isAutonomousEnabled())
                        m_roller.requestState(RollerStates.StateRollerOff);
    
    
                    }
           
                    
                }
                
                break;
                
            case StateAlignFinished:
                if(!DriverStation.isAutonomous()){
                    requestedState=AlignStates.StateDriving;
                }
                else{
                    if(requestedState != AlignStates.StateAlignCoralStationInit){

                        requestedState=AlignStates.StateAuto;


                    }
                    
                }
                currentState = requestedState; // get out of this state!
                
                //if(m_activeFollowCommand == null) m_activeFollowCommand=null;
                CommandScheduler.getInstance().cancel(m_activeFollowCommand);
                m_activeFollowCommand=null;
                break;

            case StateAbort:
                if(m_activeFollowCommand != null) {
                    CommandScheduler.getInstance().cancel(m_activeFollowCommand);


                }
                m_Handler.requestRobotState(RobotStates.StateAbort);
                requestedState=AlignStates.StateDriving;
                currentState=AlignStates.StateDriving;
                break;
            default:
                break;
        }


        // if(getStateReached()){

        //     currentState=requestedState;

        // }
        // else{
        //     currentState = RobotStates.StateBetweenStates;

        // }
        tagLog.setInteger(targetID);
        state.setString(currentState.toString());
    }
    public void abortAlign(){

        currentState = AlignStates.StateAbort;

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
