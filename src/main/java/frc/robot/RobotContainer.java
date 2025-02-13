// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.kDriverPort;
import static frc.robot.Constants.OIConstants.kOperatorPort;

import java.lang.annotation.ElementType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.StateHandler.RobotStates;
import frc.robot.commands.DriveCommandBuilder;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.PieceHandlingCommandBuilder;
import frc.robot.subsystems.PieceHandling.Elevator;
import frc.robot.subsystems.PieceHandling.Pivot;
import frc.robot.subsystems.PieceHandling.Wrist;
import frc.robot.subsystems.PieceHandling.Elevator.ElevatorStates;
import frc.robot.subsystems.PieceHandling.Pivot.PivotStates;
import frc.robot.subsystems.PieceHandling.Roller;
import frc.robot.subsystems.PieceHandling.Roller.RollerStates;
import frc.robot.subsystems.PieceHandling.Wrist.WristStates;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final Drivetrain m_drivetrain = new Drivetrain();
  private final SendableChooser<Command> autoChooser;
  private final Elevator m_elevator = new Elevator();
  private final Pivot m_pivot = new Pivot();
  private final Wrist m_wrist = new Wrist();
  private final Roller m_roller = new Roller();
  private final WL_CommandXboxController m_driver = new WL_CommandXboxController(kDriverPort);
  private final WL_CommandXboxController m_operator = new WL_CommandXboxController(kOperatorPort);
  PoseEstimation m_poseEstimation = new PoseEstimation(m_drivetrain::getYawMod, m_drivetrain::getModulePositionsInverted, m_drivetrain::getChassisSpeeds, m_driver, m_operator, m_drivetrain);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  DriveCommandBuilder m_driveBuilder = new DriveCommandBuilder(m_poseEstimation, m_drivetrain);
  public final StateHandler m_Handler = new StateHandler(m_elevator, m_wrist, m_pivot);
 
  public RobotContainer() { 
    // Configure the trigger bindings
    NamedCommands.registerCommand("ZeroGyro", new InstantCommand(m_poseEstimation::test_set));
    m_poseEstimation.addDashboardWidgets(Shuffleboard.getTab("Autos"));
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData(autoChooser);
    Shuffleboard.getTab("Autos").add(autoChooser);

    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /*
   *  DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSpeedSupplier,
      BooleanSupplier fieldRelative,
      Drivetrain drivetrain,
      PoseEstimation poseEstimation
   * 
   * 
   * 
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      new DriveWithController(
          m_driver::getLeftY,
          m_driver::getLeftX,
          m_driver::getRightX,
          () -> true,
          m_drivetrain, m_poseEstimation));
    m_driver.x().onTrue(new InstantCommand(m_drivetrain::zeroGyro).alongWith(new InstantCommand(m_drivetrain::resetToAbsolute)));
    //m_driver.a().onTrue(DriveCommandBuilder.alignToSource(m_drivetrain, m_poseEstimation));
    // m_driver.a().onTrue(new InstantCommand(() -> m_Handler.requestRobotState(RobotStates.StateCoralStationInit), m_Handler));//
    
    // m_driver.b().onTrue(new InstantCommand(() -> m_Handler.requestRobotState(RobotStates.StateL2Init), m_Handler));//
    //r.requestState(ElevatorStates.StateL2), m_elevator));
    // m_driver.a().onTrue(PieceHandlingCommandBuilder.requestL2(m_wrist, m_elevator, m_pivot));
    m_driver.leftBumper().onTrue(new InstantCommand(() -> m_roller.requestState(RollerStates.StateRollerOnBackward), m_roller)).onFalse(new InstantCommand(() -> m_roller.requestState(RollerStates.StateRollerOff), m_roller));
    m_driver.a().onTrue(DriveCommandBuilder.roughAlignToTag(21,1,0.3, m_drivetrain, m_poseEstimation));

    m_driver.rightTrigger().onTrue(new InstantCommand(() -> m_roller.requestState(RollerStates.StateRollerOnForward), m_roller)).onFalse(new InstantCommand(() -> m_roller.requestState(RollerStates.StateRollerOff), m_roller));
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //

    return autoChooser.getSelected(); // DEPRECATED!!

   // return AutoBuilder.buildAuto("SquarePauseAuto");
  }





}
