package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularAccelerationRadiansPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.signals.RobotEnableValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.StateHandler.RobotStates;
import frc.robot.subsystems.PieceHandling.Roller.RollerStates;
//import frc.robot.subsystems.SubsystemName;
//import frc.robot.subsystems.SubsystemName2;
//import frc.robot.subsystems.SubsystemName.SubsystemNameStates;
//import frc.robot.subsystems.SubsystemName2.SubsystemName2States;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.AlignHandler.AlignStates;

import static frc.robot.Constants.Swerve.*;

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

    public static Command pathfindCommand(Pose2d endPoint) { // SHOULD ONLY BE USED TO DRIVE TO DECISION PTS

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                5, 3,
                kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);

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
        StateScoringTopLeft, StateIntake1Transition,StateIntake1Init,

        StateTravelTopLeft2,
        StateTravellingTopLeft2,
        StateScoreTopLeft2,
        StateScoringTopLeft2,
    }

    public static autoPeriodicStates m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
    public static autoPeriodicStates m_autoPeriodicRequestedState = autoPeriodicStates.StateInit;
    public static BasicScoreAutoStates m_basicScoreAutoCurrentState = BasicScoreAutoStates.StateInit;
    public static BasicScoreAutoStates m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateInit;
    public static RobotContainer m_Container;

    public static void setRobotContainer(RobotContainer cont){
        m_Container=cont;
    }

    public static lineAutoStates m_lineAutoCurrentState = lineAutoStates.StateInit;
    public static lineAutoStates m_lineAutoRequestedState = lineAutoStates.StateInit;
    static GenericEntry dashEntry = Shuffleboard.getTab("Autos")
            .add("auto state", m_basicScoreAutoCurrentState.toString()).getEntry();

    public static Command m_activeFollowCommand = null;

    public static int incrementer = 0;
    private static long intakeStartTime = -1;
    private static long intakeEndTime;
    public static void reset() {
        m_lineAutoRequestedState = lineAutoStates.StateInit;

        m_lineAutoCurrentState = lineAutoStates.StateInit;
        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateInit;

        m_basicScoreAutoCurrentState = BasicScoreAutoStates.StateInit;

        m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
        intakeStartTime = -1;
    }
    static Pose2d targetPoint = Pose2d.kZero;
        // @Override
        public static void autoControlLoop() {
            // TODO: Include verification on state change here
            m_lineAutoCurrentState = m_lineAutoRequestedState;
            m_basicScoreAutoCurrentState = m_basicScoreAutoRequestedState;
            m_autoPeriodicCurrentState = m_autoPeriodicRequestedState;
            
            // dashEntry.setString(m_basicScoreAutoCurrentState.name());
            System.out.println(m_activeFollowCommand);
            dashEntry.setString(m_basicScoreAutoCurrentState.name());
            //dashEntry.setString(Integer.valueOf(incrementer).toString());
    
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
                    switch (m_basicScoreAutoCurrentState) {
    
                        case StateInit:
    
                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravelTopLeft;
    
                            break;
                        case StateIdle:
                            m_Container.m_Aligner.requestAlignState(AlignStates.StateDriving);
                            break;
                        case StateTravelTopLeft:
                            m_Container.m_roller.requestState(RollerStates.StateRollerOnForward);
                            targetPoint = new Pose2d(5, 6, Rotation2d.fromDegrees(-120));
                        m_activeFollowCommand = pathfindCommand(targetPoint);
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);

                        CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        // m_activeFollowCommand.schedule();
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravellingTopLeft;
                        incrementer++;
                        break;
                    case StateTravellingTopLeft:
                        double dist = targetPoint.getTranslation().getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());
                        if (m_activeFollowCommand.isFinished() || dist<0.75) {

                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoreTopLeft;
                        }
                        break;
                    case StateScoreTopLeft:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignRightL4Init);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoringTopLeft;
                     
                    
                        break;
                    case StateScoringTopLeft:
                        
                        if(m_Container.m_Aligner.isAllowedToDrive()){

                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake1Init;

                        }
                        break;    
                    case StateIntake1Init:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignCoralStationInit);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIntake1Transition;
                        break;
                    case StateIntake1Transition:
                        if(m_Container.m_Aligner.isAllowedToDrive() && intakeStartTime == -1){ // intakeStartTime will be -1 when not being counted
                                intakeStartTime = System.currentTimeMillis();
                            

                        }
                        else if(m_Container.m_Aligner.isAllowedToDrive()) {

                            long deltaTime = System.currentTimeMillis()-intakeStartTime;
                            if(deltaTime > 15000){
                                m_Container.m_roller.requestState(RollerStates.StateRollerOff);

                                m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIdle;

                            }

                        }                      
                        if(m_Container.m_Aligner.isAllowedToDrive() && m_Container.m_roller.isStalling())
                         // add dynamic part here
                        {
                            m_Container.m_roller.requestState(RollerStates.StateRollerOff);
                            m_basicScoreAutoRequestedState=BasicScoreAutoStates.StateTravelTopLeft2;


                        }
                        break;
                    case StateTravelTopLeft2:
                        intakeStartTime = -1;
                        m_Container.m_Handler.requestRobotState(RobotStates.StateL4Prepare1);
                        targetPoint = new Pose2d(3, 6, Rotation2d.fromDegrees(-60));
                        m_activeFollowCommand = pathfindCommand(targetPoint);
                        CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        // m_activeFollowCommand.schedule();
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateTravellingTopLeft2;
                        incrementer++;
                        break;
                    case StateTravellingTopLeft2:
                        double dist2 = targetPoint.getTranslation().getDistance(m_Container.m_poseEstimation.getCurrentPose().getTranslation());
                        if (m_activeFollowCommand.isFinished() || dist2<0.75) {

                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoreTopLeft2;
                        }
                        break;
                    case StateScoreTopLeft2:
                        m_Container.m_Aligner.requestAlignState(AlignStates.StateAlignRightL4Init);
                        m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateScoringTopLeft2;
                    
                    
                        break;
                    case StateScoringTopLeft2:
                        
                        if(m_Container.m_Aligner.isAllowedToDrive()){
                            m_Container.m_Aligner.requestAlignState(AlignStates.StateDriving);
                            m_basicScoreAutoRequestedState = BasicScoreAutoStates.StateIdle;

                        }
                        break; 
                    default:
                        break;

                }

                break;
        }

    }
}
