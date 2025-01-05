// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.DriveConstants.kTeleopMaxSpeedMetersPerSecond;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;


import frc.util.COTSFalconSwerveConstants;
import frc.util.SwerveModuleConstants;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final String kRobotIdFile = "/home/lvuser/id.txt";
  public static final String kCurrentLogFolder = "/home/lvuser/currentLogs";
  public static final double kNominalVoltage = 12.0;
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final double kFalconSensorUnitsPerRotation = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final double kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;
  public static final double kFalconFreeSpeedRotationsPerSecond = 6380.0 / 60.0;
  public static final double kSecondsPer100Ms = 0.1;

  public static final double kNeoFreeSpeedRotationsPerSecond = 5676.0 / 60.0;
  public static final double kNeo550FreeSpeedRotationsPerSecond = 11000.0 / 60.0;

  public static final double k775FreeSpeedRotationsPerSecond = 18730.0 / 60.0;

  public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
  COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; // hi asdasdsa
  } 
  public static final class AutoConstants {
    public static final double kAutoMaxSpeedMetersPerSecond = 1.5;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kAutoMaxAngularSpeedRadiansPerSecond =
        1.5 / DriveConstants.kTurningRadiusMeters;
    public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = 1 * Math.PI;

    public static final double kPAutoXController = 5;
    public static final double kIAutoXController = 0;
    public static final double kDAutoXController = 0;
    public static final double kPAutoYController = 5;
    public static final double kIAutoYController = 0;
    public static final double kDAutoYController = 0;

    public static final double kAutoXYIntegratorMaxMetersPerSecond = 0.5;
    public static final double kPAutoThetaController = 1;
    public static final double kIAutoThetaController = 0;
    public static final double kAutoThetaIntegratorMaxRadiansPerSecond = 0.2;
    public static final double kDAutoThetaController = 0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kAutoMaxAngularSpeedRadiansPerSecond,
            kAutoMaxAngularAccelerationRadiansPerSecondSquared);
  }
  public static final class DriveConstants {

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Swerve.widthBetweenModules/ 2, Swerve.lengthBetweenModules / 2),
            new Translation2d(Swerve.lengthBetweenModules / 2, -Swerve.widthBetweenModules / 2),
            new Translation2d(-Swerve.lengthBetweenModules / 2, Swerve.widthBetweenModules / 2),
            new Translation2d(-Swerve.lengthBetweenModules / 2, -Swerve.widthBetweenModules / 2));

    public static final double kTurningRadiusMeters =
        Math.sqrt(Math.pow(Swerve.lengthBetweenModules / 2, 2) + Math.pow(Swerve.widthBetweenModules / 2, 2));

    // Max speed teleoperated
    public static final double kTeleopMaxSpeedMetersPerSecond = 3; // meters per second
    public static final double kTeleopMaxAngularSpeedRadiansPerSecond =
        4; // radians per second

    public static final double kDriveTolerance = .1;

    public static final double kTeleopMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTeleopMaxAngularAccelerationRadiansPerSecondSquared = 1.5 * Math.PI;

    public static final int kPoseHistoryCapacity = 500;

    public static final double kPRotation = 2;
    public static final double kRotationTolerance = 3;
  }


  public static final class Swerve 
  {
    public static final int pigeonID = 9;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double lengthBetweenModules = 0.5842;
    public static final double widthBetweenModules = 0.5842;
    public static final double driveRadius = 0.413091;
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    // public static RobotConfig pathplannerRobotConfig;

    //new RobotConfig(null, null, null, null)
    // public static final HolonomicPathFollowerConfig kPathFollowingConfig = new HolonomicPathFollowerConfig(// HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                     new PIDConstants(2, 0.0, 0), // Translation PID constants
    //                     new PIDConstants(2, 0, 0), // Rotation PID constants
    //                     6, // Max module speed, in m/s
    //                     driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //                     new ReplanningConfig());
  
    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(Swerve.lengthBetweenModules / 2.0, -Swerve.widthBetweenModules / 2.0),
        new Translation2d(Swerve.lengthBetweenModules / 2.0, Swerve.widthBetweenModules / 2.0),
        new Translation2d(-Swerve.lengthBetweenModules / 2.0, Swerve.widthBetweenModules / 2.0),
        new Translation2d(-Swerve.lengthBetweenModules / 2.0, -Swerve.widthBetweenModules / 2.0));

    
    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 40;
    public static final int anglePeakCurrentLimit = 80;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    // public static final double angleKP = chosenModule.angleKP;
    // public static final double angleKI = chosenModule.angleKI;
    // public static final double angleKD = chosenModule.angleKD;
    // public static final double angleKF = chosenModule.angleKF;

    public static final double angleKP = 30;
    public static final double angleKI = 0;
    public static final double angleKD = .05;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 2.5;
    public static final double driveKI = 3;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 6; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 3; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.36279296875);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.42529296875);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Right Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.1552734375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Left Module - Module 3- */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.191162109375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }
  }
}
