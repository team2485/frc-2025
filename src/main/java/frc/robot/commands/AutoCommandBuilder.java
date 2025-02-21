package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularAccelerationRadiansPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.subsystems.SubsystemName;
//import frc.robot.subsystems.SubsystemName2;
//import frc.robot.subsystems.SubsystemName.SubsystemNameStates;
//import frc.robot.subsystems.SubsystemName2.SubsystemName2States;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;

public class AutoCommandBuilder {

    public static Command createPathCommand(String path)
    {
        PathPlannerPath followPath;
        try{
            followPath = PathPlannerPath.fromPathFile(path);
        }
        catch(IOException exception)
        {
            return null;
        }
        catch(ParseException exception)
        {
            return null;
        }
        return AutoBuilder.followPath(followPath);
    }
    public static Command pathfindCommand(Pose2d endPoint){ // SHOULD ONLY BE USED TO DRIVE TO DECISION PTS

        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                1, 1,
                kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);

        // PathFindHolonomic is confirmed functional without collisions avoidance, AutoBuilder must be used to avoid collision
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(endPoint, constraints, 0.0);
        return pathfindingCommand;

    }

    public enum autoPeriodicStates{
        StateInit,
        lineAuto,
        BasicScoreAuto,
    }
    public enum lineAutoStates{
        StateInit,
        StateIdle,
        StateFollowLine,
        StateFollowingLine,
        StateFollowLine2,
        StateFollowingLine2
    }
    public enum BasicScoreAutoStates{

        StateInit,
        StateIdle,
        StateTravelTopLeft,
        StateTravellingTopLeft,
        StateScoreTopLeft,
        StateScoringTopLeft,
        

    }

    public static autoPeriodicStates m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
    public static autoPeriodicStates m_autoPeriodicRequestedState = autoPeriodicStates.StateInit;
    public static BasicScoreAutoStates m_basicScoreAutoCurrentState = BasicScoreAutoStates.StateInit;
    public static BasicScoreAutoStates m_basicScoreAutoRequestedtate = BasicScoreAutoStates.StateInit;
    
    public static lineAutoStates m_lineAutoCurrentState = lineAutoStates.StateInit;
    public static lineAutoStates m_lineAutoRequestedState = lineAutoStates.StateInit;
    static GenericEntry dashEntry = Shuffleboard.getTab("Autos").add("auto state", m_lineAutoCurrentState.toString()).getEntry();
    
    public static Command m_activeFollowCommand = null;

    public static void reset(){
        m_lineAutoRequestedState=lineAutoStates.StateInit;

        m_lineAutoCurrentState=lineAutoStates.StateInit;
        m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
    }
    //@Override
    public static void autoControlLoop(){
        // TODO: Include verification on state change here
        m_lineAutoCurrentState = m_lineAutoRequestedState;
        dashEntry.setString(m_lineAutoCurrentState.name());

        System.out.println(m_activeFollowCommand);
        switch(m_autoPeriodicCurrentState){
            case StateInit:
                m_autoPeriodicRequestedState = autoPeriodicStates.lineAuto; // desired auto can go here based on chooser :)
            case lineAuto:

                switch(m_lineAutoCurrentState){
                    case StateInit:
                        m_lineAutoRequestedState = lineAutoStates.StateFollowLine;
                        break;
                    case StateIdle:
                        break;
                    case StateFollowLine:
                        m_activeFollowCommand = createPathCommand("line");
                        m_activeFollowCommand.schedule();
                        // CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        m_lineAutoRequestedState = lineAutoStates.StateFollowingLine;
                        break;
                    case StateFollowingLine:
                        if(m_activeFollowCommand.isFinished()){

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
                        if(m_activeFollowCommand.isFinished()){

                            // doneso
                            m_lineAutoRequestedState = lineAutoStates.StateIdle;
                        }
                    
                        break;
                        
                    
                }

            case BasicScoreAuto:
                switch(m_basicScoreAutoCurrentState){

                    case StateInit:
                        m_basicScoreAutoCurrentState=BasicScoreAutoStates.StateTravelTopLeft;

                        break;
                    case StateIdle:
                        break;  
                    case StateTravelTopLeft:
                        Pose2d topLeftPoint = new Pose2d(3,6,Rotation2d.fromDegrees(-60));
                        m_activeFollowCommand = pathfindCommand(topLeftPoint);
                        m_activeFollowCommand.schedule();;
                        m_basicScoreAutoCurrentState=BasicScoreAutoStates.StateTravellingTopLeft;
                    case StateTravellingTopLeft:
                        if(m_activeFollowCommand.isFinished()){



                        }
                    case StateScoreTopLeft:
                        
                    



                }
                    

            }

    }
}

