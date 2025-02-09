// TODO: Port to new pathplanner stuff


package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kTeleopMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularAccelerationRadiansPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.DriveConstants.kTeleopMaxSpeedMetersPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.type.ClassKey;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// Imports go here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PieceHandling.Elevator;
import frc.robot.subsystems.PieceHandling.Pivot;
import frc.robot.subsystems.PieceHandling.Wrist;
import frc.robot.subsystems.PieceHandling.Elevator.ElevatorStates;
import frc.robot.subsystems.PieceHandling.Pivot.PivotStates;
import frc.robot.subsystems.PieceHandling.Wrist.WristStates;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import static frc.robot.Constants.Swerve.*;

public class PieceHandlingCommandBuilder {



    public PieceHandlingCommandBuilder() {



    }
    public static Command requestL1(Wrist m_wrist, Elevator m_elevator, Pivot m_Pivot){
        Command wristCom = new InstantCommand(()->m_wrist.requestState(WristStates.StateL1),m_wrist);
        Command elevatorCommand = new InstantCommand(()->m_elevator.requestState(ElevatorStates.StateL1),m_elevator);
        
        Command pivotCommand = new InstantCommand(()->m_Pivot.requestState(PivotStates.StateL1),m_Pivot);

        
        
        return wristCom.alongWith(elevatorCommand.alongWith(pivotCommand));
    }

    public static Command requestL2(Wrist m_wrist, Elevator m_elevator, Pivot m_Pivot){
        Command wristCom = new InstantCommand(()->m_wrist.requestState(WristStates.StateL2),m_wrist);
        Command elevatorCommand = new InstantCommand(()->m_elevator.requestState(ElevatorStates.StateL2),m_elevator);
        
        Command pivotCommand = new InstantCommand(()->m_Pivot.requestState(PivotStates.StateL2),m_Pivot);

        
        
        return wristCom.alongWith(elevatorCommand.alongWith(pivotCommand));
    }
    // public static Command elevatorTestUp(){



    // }

}
