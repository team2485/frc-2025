package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularAccelerationRadiansPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;

import java.io.IOException;
import java.util.concurrent.BlockingDeque;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.signals.RobotEnableValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.StateHandler.RobotStates;
import frc.robot.subsystems.PieceHandling.Roller.RollerStates;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.AlignHandler.AlignStates;


public class AutoCommandBuilder {

    public static Command createPathCommand(String path) {
        PathPlannerPath followPath;
        try {
            followPath = PathPlannerPath.fromPathFile(path);
        } catch (IOException exception) {
            return null;
        } catch (ParseException exception) {
            return null;
        }
        return AutoBuilder.followPath(followPath);
    }
    static boolean isExtended = false;
    public static Command pathfindCommand(Pose2d endPoint) { // SHOULD ONLY BE USED TO DRIVE TO DECISION PTS

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation

        // Create the constraints to use while pathfinding
        PathConstraints constraints;
        if(!isExtended){

            constraints     = new PathConstraints(
                6, 3.5,
                kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);


        }else{

            
            constraints     = new PathConstraints(
                6, 3.5,
                kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);

        }
      
        // PathFindHolonomic is confirmed functional without collisions avoidance,
        // AutoBuilder must be used to avoid collision

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(endPoint, constraints, 0.0);
        return pathfindingCommand;

    }

    public enum autoPeriodicStates {
        StateInit,
        lineAuto,
        BasicScoreAuto,
        BasicMidScoreAuto,
    }

    public enum lineAutoStates {
        StateInit,
        StateIdle,
        StateFollowLine,
        StateFollowingLine,
        StateFollowLine2,
        StateFollowingLine2
    }

    public enum BasicScoreAutoStates {

        StateInit,
        StateIdle,
        StateTravelTopLeft,
        StateTravellingTopLeft,
        StateScoreTopLeft,
        StateScoringTopLeft, StateIntake1Transition, StateIntake1Init,
        StateIntake2Init,
        StateTravelTopLeft2,
        StateTravellingTopLeft2,
        StateScoreTopLeft2,
        StateScoringTopLeft2, StateIntake2Transition, StateAbortInit, StateAbort, StateScore3, StateScoring3, StateDone,
    }

    public enum BasicMidScoreAutoStates {
        StateInit,
        StateIdle,
        StateTravelMiddle,
        StateTravellingMiddle,
        StateScoreMiddle,
        StateScoringMiddle, 
        StateIntake1Transition, StateIntake1Init,
        StateTravelMiddleAlgae,
        StateTravellingMiddleAlgae,
        StateTravelBarge,
        StateTravellingBarge,
        StateScoreBarge,
        StateScoringBarge,
        StateIntake2Transition, StateAbortInit,
    }

    public static autoPeriodicStates m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
    public static autoPeriodicStates m_autoPeriodicRequestedState = autoPeriodicStates.StateInit;

    public static BasicScoreAutoStates m_basicScoreAutoCurrentState = BasicScoreAutoStates.StateInit;
    public static BasicScoreAutoStates m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateInit;

    public static BasicMidScoreAutoStates m_basicMidScoreAutoCurrentState =  BasicMidScoreAutoStates.StateInit;
    public static BasicMidScoreAutoStates m_basicMidScoreAutoRequestedState =  BasicMidScoreAutoStates.StateInit;

    public static RobotContainer m_Container;

    public static void setRobotContainer(RobotContainer cont) {
        m_Container = cont;
    }

    public static lineAutoStates m_lineAutoCurrentState = lineAutoStates.StateInit;
    public static lineAutoStates m_lineAutoRequestedState = lineAutoStates.StateInit;
    static GenericEntry dashEntry = Shuffleboard.getTab("Autos")
            .add("auto state", m_basicScoreAutoCurrentState.toString()).getEntry();
    static GenericEntry shouldRunTop = Shuffleboard.getTab("Autos").add("Should Run Topside?",true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static Command m_activeFollowCommand = null;

    public static int incrementer = 0;
    private static long intakeStartTime = -1;
    private static long intakeEndTime;
    private static boolean isOnRed;
    private static boolean runsTop;// = false; // TODO: CHANGE THIS BASED ON WHERE U WANNA RUN AUTO
    public static void reset() {
        runsTop = shouldRunTop.getBoolean(true);
        isOnRed = m_Container.m_poseEstimation.getFieldConstants().isOnRed();
        m_lineAutoRequestedState = lineAutoStates.StateInit;

        m_lineAutoCurrentState = lineAutoStates.StateInit;
        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateInit;

        m_basicScoreAutoCurrentState = BasicScoreAutoStates.StateInit;

        m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
        intakeStartTime = -1;
    }

    public static void forceTeleOp() {

        m_lineAutoRequestedState = lineAutoStates.StateIdle;

        m_lineAutoCurrentState = lineAutoStates.StateIdle;
        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIdle;

        m_basicScoreAutoCurrentState = BasicScoreAutoStates.StateIdle;
        if (m_activeFollowCommand != null) {

            m_activeFollowCommand.cancel();

        }
        m_Container.m_Aligner.forceState(AlignStates.StateDriving);

    }

    static Pose2d targetPoint = Pose2d.kZero;

    // @Override
    public static void autoControlLoop() {
        // TODO: Include verification on state change here
        m_lineAutoCurrentState = m_lineAutoRequestedState;
        m_basicScoreAutoCurrentState = m_basicScoreAutoRequestedState;
        m_autoPeriodicCurrentState = m_autoPeriodicRequestedState;
        m_basicMidScoreAutoCurrentState = m_basicMidScoreAutoRequestedState;
        // dashEntry.setString(m_basicScoreAutoCurrentState.name());
        System.out.println(m_activeFollowCommand);
        dashEntry.setString(m_basicScoreAutoRequestedState.name());
        // dashEntry.setString(Integer.valueOf(incrementer).toString());

        switch (m_autoPeriodicCurrentState) {
            case StateInit:
                m_Container.m_Aligner.requestAlignState(AlignStates.StateAuto);
                m_autoPeriodicRequestedState = autoPeriodicStates.BasicScoreAuto; // desired auto can go here based on
                                                                                  // chooser :)
                break;
            case lineAuto:

                switch (m_lineAutoCurrentState) {
                    case StateInit:
                        m_lineAutoRequestedState = lineAutoStates.StateFollowLine;
                        break;
                    case StateIdle:
                        break;
                    case StateFollowLine:
                        m_activeFollowCommand = createPathCommand("line");
                        m_activeFollowCommand.schedule();
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);
                        // CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        m_lineAutoRequestedState = lineAutoStates.StateFollowingLine;
                        break;
                    case StateFollowingLine:
                        if (m_activeFollowCommand.isFinished()) {

                            // change states
                            // m_lineAutoRequestedState = lineAutoStates.StateFollowLine2;
                            m_lineAutoRequestedState = lineAutoStates.StateFollowLine2;
                        }
                        break;
                    case StateFollowLine2:
                        m_activeFollowCommand = createPathCommand("line2");
                        m_activeFollowCommand.schedule();
                        m_lineAutoRequestedState = lineAutoStates.StateFollowingLine2;
                        break;
                    case StateFollowingLine2:
                        if (m_activeFollowCommand.isFinished()) {

                            // doneso
                            m_lineAutoRequestedState = lineAutoStates.StateIdle;
                        }

                        break;

                }
                break;

            case BasicScoreAuto:
                if(m_Container.m_Handler.getCurrentState() == RobotStates.StateCoralStationFinal){
                    isExtended = false;
                

                }
                else{
                    isExtended=true;
                }
                switch (m_basicScoreAutoCurrentState) {
                    
                    case StateInit:

                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravelTopLeft;
                        intakeStartTime = -1;

                        break;
                    case StateIdle:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAuto);
                        break;
                    case StateTravelTopLeft:
                        m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);
                        // isOnRed = m_Container.m_poseEstimation.getFieldConstants().isOnRed();



                        // targetPoint = new Pose2d(5, 6, Rotation2d.fromDegrees(-120));
                        if(!isOnRed){
                            if(runsTop){


                                targetPoint = new Pose2d(5.5, 5.5, Rotation2d.fromDegrees(-120));

                            }
                            else{

                                targetPoint = new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(120));


                            }

                        }
                        else{
                            if(runsTop){
                                targetPoint = new Pose2d(17.55-5.5, 5.5, Rotation2d.fromDegrees(-60));
                            }else{


                                targetPoint = new Pose2d(17.55-5.5 , 2.5, Rotation2d.fromDegrees(60));

                            }

                        }


                        m_activeFollowCommand = pathfindCommand(targetPoint);
                        
                       // m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);

                        // CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        incrementer++;
                        // intakeStartTime = -1;
                        intakeStartTime = System.currentTimeMillis();
                        // m_activeFollowCommand.schedule();
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravellingTopLeft;

                        break;
                    case StateTravellingTopLeft:
                        double dist = targetPoint.getTranslation()
                                .getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());


                        // if (intakeStartTime == -1) { // intakeStartTime will be -1 when not being counted
                            

                        // }

                        long deltaTime = System.currentTimeMillis() - intakeStartTime;
                        m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                  
                        if (deltaTime > 400)
                        // add dynamic part here
                        {

                            // m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                            // m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravelTopLeft2;
                            m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);
                            m_activeFollowCommand.cancel();
                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoreTopLeft;
        
                        }

                        
                        // if (m_activeFollowCommand.isFinished()){// || dist < 0.75) {
                        // }
                        break;
                    case StateScoreTopLeft:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignRightL4Init);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoringTopLeft;

                        break;
                    case StateScoringTopLeft:

                        if (m_Container.m_Aligner.isAllowedToDrive()) {

                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake1Init;

                        }
                        break;
                    case StateIntake1Init:
                       m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignCoralStationInit);

                       // put in the command here that makes it go forward;
                         
                       intakeStartTime = -1;
                        
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake1Transition;
                        break;
                    case StateIntake1Transition:
                        int targetCoralID = DriveCommandBuilder.findNearestSourceId(m_Container.m_poseEstimation,m_Container.m_drivetrain);
                        Pose2d forwardPosCoral = new Pose2d(179*Constants.kInchesToMeters, 159 * Constants.kInchesToMeters, Rotation2d.kZero);// DriveCommandBuilder.convertAprilTag(targetCoralID2, 0.8, 0, m_Container.m_drivetrain, m_Container.m_poseEstimation,true);
                        if(isOnRed){

                            forwardPosCoral = new Pose2d(Constants.VisionConstants.kFieldLengthMeters - (179*Constants.kInchesToMeters), 159 *Constants.kInchesToMeters, Rotation2d.kZero);

                        } 
                        
                       // Pose2d forwardPosCoral = DriveCommandBuilder.convertAprilTag(targetCoralID, 0.8, 0, m_Container.m_drivetrain, m_Container.m_poseEstimation,true);
                        double distIntake = forwardPosCoral.getTranslation()
                        .getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());
                        if(distIntake >2){

                            m_Container.m_Handler.requestRobotState(RobotStates.StateCoralStationInit);

                        }
                        if (m_Container.m_Aligner.isAllowedToDrive()) {
                            // m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                            if (intakeStartTime == -1) { // intakeStartTime will be -1 when not being counted
                                intakeStartTime = System.currentTimeMillis();

                            }

                            long deltaTime2 = System.currentTimeMillis() - intakeStartTime;
                            m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                            if (m_Container.m_roller.isStalling() && deltaTime2 > 1500)
                            // add dynamic part here
                            {

                                m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                                m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravelTopLeft2;

                            }

                            if (deltaTime2 > 5000) {
                                m_Container.m_roller.requestState(RollerStates.StateRollerOff);

                                m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateAbortInit; // ABORT

                            }

                        }

                        break;
                    case StateTravelTopLeft2:
                        intakeStartTime = -1;
                        if(!isOnRed){ 
                            if(runsTop){
                                targetPoint = new Pose2d(3, 6, Rotation2d.fromDegrees(-60));
                            }
                            else{
                                targetPoint = new Pose2d(3, 2, Rotation2d.fromDegrees(60));
                            }


                        }
                        else{
                            if(runsTop){
                            targetPoint = new Pose2d(17.55-3 , 6, Rotation2d.fromDegrees(-120));
                            }
                            else{
                                targetPoint = new Pose2d(17.55-3, 2, Rotation2d.fromDegrees(120));
                            }

                        }
                        m_activeFollowCommand = pathfindCommand(targetPoint);
                        // CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        // m_activeFollowCommand.schedule();
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);

                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravellingTopLeft2;
                        incrementer++;
                        break;
                    case StateTravellingTopLeft2:
                        double dist2 = targetPoint.getTranslation()
                                .getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());
                        // if (m_activeFollowCommand.isFinished() ) {
                            m_activeFollowCommand.cancel();
                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoreTopLeft2;
                        // }
                        break;
                    case StateScoreTopLeft2:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignLeftL4Init);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoringTopLeft2;

                        break;
                    case StateScoringTopLeft2:

                        if (m_Container.m_Aligner.isAllowedToDrive()) {
                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake2Init;

                        }
                        break;
                    case StateIntake2Init:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignCoralStationInit);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake2Transition;
                        intakeStartTime = -1;
                        break;
                    case StateIntake2Transition:
                        int targetCoralID2 = DriveCommandBuilder.findNearestSourceId(m_Container.m_poseEstimation,m_Container.m_drivetrain);
                        
                        Pose2d forwardPosCoral2 = new Pose2d(179*Constants.kInchesToMeters, 159 * Constants.kInchesToMeters, Rotation2d.kZero);// DriveCommandBuilder.convertAprilTag(targetCoralID2, 0.8, 0, m_Container.m_drivetrain, m_Container.m_poseEstimation,true);
                        if(isOnRed){

                            forwardPosCoral2 = new Pose2d(Constants.VisionConstants.kFieldLengthMeters - (179*Constants.kInchesToMeters), 159 *Constants.kInchesToMeters, Rotation2d.kZero);

                        } 
                        
                        
                        double distIntake2 = forwardPosCoral2.getTranslation()
                        .getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());
                        if(distIntake2 > 1.6){

                            m_Container.m_Handler.requestRobotState(RobotStates.StateCoralStationInit);

                        }
                        if (m_Container.m_Aligner.isAllowedToDrive()) {
                            // m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                            if (intakeStartTime == -1) { // intakeStartTime will be -1 when not being counted
                                intakeStartTime = System.currentTimeMillis();

                            }

                            long deltaTime3 = System.currentTimeMillis() - intakeStartTime;
                            m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                            if (m_Container.m_roller.isStalling() && deltaTime3 > 1500)
                            // add dynamic part here
                            {

                                m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                                m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScore3;

                            }

                            if (deltaTime3 > 5000) {
                                m_Container.m_roller.requestState(RollerStates.StateRollerOff);

                                m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateAbortInit; // ABORT

                            }

                        }

                
                        break;
                    case StateScore3:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignRightL4Init);
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Init);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoring3;
                        break;
                    case StateScoring3:
                        
                        if (m_Container.m_Aligner.isAllowedToDrive()) {
                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateDone;

                        }
                        break;
                    case StateDone:
                        int targetCoralID3 = DriveCommandBuilder.findNearestSourceId(m_Container.m_poseEstimation,m_Container.m_drivetrain);
                        Pose2d forwardPosCoral3 = new Pose2d(179*Constants.kInchesToMeters, 159 * Constants.kInchesToMeters, Rotation2d.kZero);// DriveCommandBuilder.convertAprilTag(targetCoralID2, 0.8, 0, m_Container.m_drivetrain, m_Container.m_poseEstimation,true);
                        if(isOnRed){

                            forwardPosCoral3 = new Pose2d(Constants.VisionConstants.kFieldLengthMeters - (179*Constants.kInchesToMeters), 159 *Constants.kInchesToMeters, Rotation2d.kZero);

                        }
                       // Pose2d forwardPosCoral3 = DriveCommandBuilder.convertAprilTag(targetCoralID3, 0.8, 0, m_Container.m_drivetrain, m_Container.m_poseEstimation,true);
                        double distIntake3 = forwardPosCoral3.getTranslation()
                        .getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());
                        if(distIntake3 > 2){

                            m_Container.m_Handler.requestRobotState(RobotStates.StateCoralStationInit);

                        }
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignCoralStationInit);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake2Transition;
                        intakeStartTime = -1;
                        
                        break;
                    case StateAbortInit:
                        m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                        if(!isOnRed){

                            targetPoint = new Pose2d(5, 7, Rotation2d.fromDegrees(-120));


                        }
                        else{

                            targetPoint = new Pose2d(17.55-5, 7, Rotation2d.fromDegrees(-120));
                            

                        }
                        m_activeFollowCommand = pathfindCommand(targetPoint);
                        // m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);

                        CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        // m_activeFollowCommand.schedule();
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateAbort;
                    
                        break;
                    case StateAbort:
                        if(m_activeFollowCommand.isFinished()){

                            m_basicScoreAutoRequestedState=BasicScoreAutoStates.StateIdle;


                        }
                        break;
                    default:
                        break;

                }

                break;


                //middle auto


            case BasicMidScoreAuto:
                switch (m_basicMidScoreAutoCurrentState) {

                    case StateInit:

                        m_basicMidScoreAutoRequestedState = BasicMidScoreAutoStates.StateScoreMiddle;
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);
                        intakeStartTime = -1;

                        break;
                    case StateIdle:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAuto);
                        break;

                    case StateScoreMiddle:
                        if(m_Container.m_Handler.getCurrentState() == RobotStates.StateL4Prepare2){

                            m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignLeftL4Init);
                            m_basicMidScoreAutoRequestedState = BasicMidScoreAutoStates.StateScoringMiddle;
                            

                        }
                        break;
                    
                    case StateScoringMiddle:
                        if (m_Container.m_Aligner.isAllowedToDrive()) {
                        m_basicMidScoreAutoRequestedState = BasicMidScoreAutoStates.StateIntake1Init;
                        }
                        break;
        
                    case StateIntake1Init:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignAlgaeL2Init);
                        // intakeStartTime = -1;
                        
                        m_basicMidScoreAutoRequestedState = BasicMidScoreAutoStates.StateIntake1Transition;
                        break;
                    case StateIntake1Transition:
                        if (m_Container.m_Aligner.isAllowedToDrive()) {
                            // m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                            // if (intakeStartTime == -1) { // intakeStartTime will be -1 when not being counted
                            //     intakeStartTime = System.currentTimeMillis();

                            // }

                            // long deltaTime2 = System.currentTimeMillis() - intakeStartTime;
                            // m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);

                            // if (m_Container.m_roller.isStalling() && deltaTime2 > 1000)
                            // add dynamic part here
                            // {
                                // m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                                m_basicMidScoreAutoRequestedState = BasicMidScoreAutoStates.StateTravelBarge;

                            // }

                            // if (deltaTime2 > 150000) {
                            //     m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                            //     m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateAbortInit; // ABORT

                            // }
                        }
                        break;
                    case StateTravelBarge:
                        intakeStartTime = -1;
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);
                        if(!isOnRed){ 
                            targetPoint = new Pose2d(7.575, 6.118, Rotation2d.fromDegrees(0));
                        }
                        else{
                            targetPoint = new Pose2d(10, 2, Rotation2d.fromDegrees(0));
                        }

                        
                        m_activeFollowCommand = pathfindCommand(targetPoint);
                        CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        // m_activeFollowCommand.schedule();
                        m_basicMidScoreAutoRequestedState = BasicMidScoreAutoStates.StateAbortInit;
                        incrementer++;
                        break;

                }

                break;
        }

    }
}
