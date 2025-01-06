// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants.OIConstants;
import static frc.robot.Constants.OIConstants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final Drivetrain m_drivetrain = new Drivetrain();

  private final WL_CommandXboxController m_driver = new WL_CommandXboxController(kDriverPort);
  private final WL_CommandXboxController m_operator = new WL_CommandXboxController(kOperatorPort);
  PoseEstimation m_poseEstimation = new PoseEstimation(m_drivetrain::getYawMod, m_drivetrain::getModulePositions, m_drivetrain::getChassisSpeeds, m_driver, m_operator, m_drivetrain);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    // Configure the trigger bindings
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
       // TODO: Placeholder
    };
  }
}
