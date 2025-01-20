package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PieceHandling.GeneralRoller;
import frc.robot.subsystems.PieceHandling.GeneralRoller.GeneralRollerStates;
import frc.robot.subsystem.PieceHandling.Pivot;
import frc.robot.subsystem.PieceHandling.Pivot.PivotStates;
import frc.robot.subsystem.PieceHandling.Elevator;
import frc.robot.subsystem.PieceHandling.Elevator.ElevatorStates;
import frc.robot.subsystem.PieceHandling.Wrist;
import frc.robot.subsystem.PieceHandling.Wrist.WristStates;

public class PieceHandlingCommandBuilder {

    public static Command generalRollerRunIntake(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateIntake), generalRoller);
    }
    
    public static Command generalRollerRunOuttake(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateOuttake), generalRoller);
    }

    public static Command generalRollerStop(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateOff), generalRoller);
    }    

    public static Command L1(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.L1); 
            wrist.requestState(WristStates.L1); 
            elevator.requestState(ElevatorStates.L1);
            }, 
            pivot , wrist, elevator);
    }
    public static Command L2(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.L2); 
            wrist.requestState(WristStates.L2); 
            elevator.requestState(ElevatorStates.L2);
            }, 
            pivot , wrist, elevator);
    }

    public static Command L3(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.L3); 
            wrist.requestState(WristStates.L3); 
            elevator.requestState(ElevatorStates.L3);
            }, 
            pivot , wrist, elevator);
    }

    public static Command L4(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.L4); 
            wrist.requestState(WristStates.L4); 
            elevator.requestState(ElevatorStates.L4);
            }, 
            pivot , wrist, elevator);
    }

    public static Command CoralIntake(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.CoralIntake); 
            wrist.requestState(WristStates.CoralIntake); 
            elevator.requestState(ElevatorStates.CoralIntake);
            }, 
            pivot , wrist, elevator);
    }

    public static Command LowAlgae(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.LowAlgae); 
            wrist.requestState(WristStates.LowAlgae); 
            elevator.requestState(ElevatorStates.LowAlgae);
            }, 
            pivot , wrist, elevator);
    }

    public static Command HighAlgae(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.HighAlgae); 
            wrist.requestState(WristStates.HighAlgae); 
            elevator.requestState(ElevatorStates.HighAlgae);
            }, 
            pivot , wrist, elevator);
    }

    public static Command LollipopAlgae(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.LollipopAlgae); 
            wrist.requestState(WristStates.LollipopAlgae); 
            elevator.requestState(ElevatorStates.LollipopAlgae);
            }, 
            pivot , wrist, elevator);
    }

    public static Command Processor(Pivot pivot, Wrist wrist, Elevator elevator) {
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.Processor); 
            wrist.requestState(WristStates.Processor); 
            elevator.requestState(ElevatorStates.Processor);
            }, 
            pivot , wrist, elevator);
    }
    public static Command Net(Pivot pivot, Wrist wrist, Elevator elevator) {   
        return new InstantCommand(() -> {
            pivot.requestState(PivotStates.Net); 
            wrist.requestState(WristStates.Net); 
            elevator.requestState(ElevatorStates.Net);
            }, 
            pivot , wrist, elevator);
    }


}