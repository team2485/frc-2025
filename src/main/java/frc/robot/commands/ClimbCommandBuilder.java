package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climb.Climber;
import frc.robot.subsystems.Climb.Climber.ClimberStates;
import frc.robot.subsystems.PieceHandling.GeneralRoller;
import frc.robot.subsystems.PieceHandling.GeneralRoller.GeneralRollerStates;
import frc.robot.subsystems.PieceHandling.Pivot;
import frc.robot.subsystems.PieceHandling.Pivot.PivotStates;

public class ClimbCommandBuilder {

        public static Command upPosition(Climber climber) {
            Command command = new InstantCommand(()-> climber.requestState(ClimberStates.StateUp), climber);
            return command;
        }

        public static Command climb(Climber climber) {
            Command command = new SequentialCommandGroup(
                            new InstantCommand(()-> climber.requestState(ClimberStates.StateDownPosition), climber)
            );   
            return command;
        }
}