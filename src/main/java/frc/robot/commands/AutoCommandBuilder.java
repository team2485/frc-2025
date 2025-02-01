package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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

    public enum autoPeriodicStates{
        StateInit,
        lineAuto,
    }
    public enum lineAutoStates{
        StateInit,
        StateIdle,
        StateFollowLine,
        StateFollowingLine,
        StateFollowLine2,
        StateFollowingLine2
    }

    public static autoPeriodicStates m_autoPeriodicCurrentState = autoPeriodicStates.StateInit;
    public static autoPeriodicStates m_autoPeriodicRequestedState = autoPeriodicStates.StateInit;

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
                m_autoPeriodicRequestedState = autoPeriodicStates.lineAuto;
            case lineAuto:
                switch(m_lineAutoCurrentState){
                    case StateInit:
                        m_lineAutoRequestedState = lineAutoStates.StateFollowLine;
                        break;
                    case StateIdle:
                        break;
                    case StateFollowLine:
                        m_activeFollowCommand = createPathCommand("line");
                        CommandScheduler.getInstance().schedule(m_activeFollowCommand);
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
                        CommandScheduler.getInstance().schedule(m_activeFollowCommand);
                        m_lineAutoRequestedState = lineAutoStates.StateFollowingLine2;
                        break;
                    case StateFollowingLine2:
                        if(m_activeFollowCommand.isFinished()){

                            // doneso
                            m_lineAutoRequestedState = lineAutoStates.StateIdle;
                        }
                        break;
                        
                    
                }

        }

    }
}

