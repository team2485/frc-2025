package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PieceHandling.GeneralRoller;
import frc.robot.subsystems.PieceHandling.GeneralRoller.GeneralRollerStates;

public class PieceHandlingCommandBuilder {

    public static Command generalRollerRunForward(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateForward), generalRoller);
    }
    
    public static Command generalRollerRunReverse(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateReverse), generalRoller);
    }

    public static Command generalRollerStop(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateOff), generalRoller);
    }    

}