// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.util.COTSFalconSwerveConstants;
import frc.util.SwerveModuleConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Swerve.angleGearRatio;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;


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
  public static final double kInchesToMeters = 0.0254;
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

  public static final COTSFalconSwerveConstants chosenModule =
  COTSFalconSwerveConstants.SDSMK4n(COTSFalconSwerveConstants.driveGearRatios.SDSMK4n_L1); //TODO: get this to the right number!!

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.05;
    public static final double kDriverLeftXDeadband = 0.05;
    public static final double kDriverLeftYDeadband = 0.05;

    public static final double kTriggerThreshold = 0.1;
  }

  public interface FieldConstants {
    public List<AprilTag> getAprilTagList();
    public AprilTag[] getReefTags();
    public boolean isOnRed();
    public Pose2d getUpperPickupPos();
    public Pose2d getLowerPickupPos();
  }

  public static final class VisionConstants {
    public static final String kCameraName = "photonvision";

    // old constraints (might want to use again)

    public static final TrapezoidProfile.Constraints kXConstraints = new TrapezoidProfile.Constraints(1, 2);
    public static final TrapezoidProfile.Constraints kYConstraints = new TrapezoidProfile.Constraints(.5, 2);
    public static final TrapezoidProfile.Constraints kOmegaConstraints = new TrapezoidProfile.Constraints(3, 8);

    public static final double kTranslationTolerance = 0.02;
    public static final double kThetaTolerance = Units.degreesToRadians(0);

    // TODO: tune!
    public static final TrapezoidProfile.Constraints kDefaultXYContraints = new TrapezoidProfile.Constraints(
        Swerve.maxSpeed * 0.3,
        Swerve.maxAngularVelocity);

    public static final TrapezoidProfile.Constraints kDefaultOmegaConstraints = new TrapezoidProfile.Constraints(
        Swerve.maxAngularVelocity * 0.2,
        Swerve.maxAngularVelocity);

    // TODO: tune!
    public static final double X_kP = 1.25;
    public static final double X_kI = 0.3;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 1.25;
    public static final double Y_kI = 0.3;
    public static final double Y_kD = 0.0;  

    public static final double THETA_kP = 1.5;
    public static final double THETA_kI = 0.5;
    public static final double THETA_kD = 0.15;

    // TODO: ensure validity of measurements
    public static final Transform3d kRobotToCameraLeft = new Transform3d(new Translation3d(0.3429, 0.27305, 0.187325),
        new Rotation3d(0,.1745,0)); 
    public static final Transform3d kRobotToCameraRight = new Transform3d(new Translation3d(0.3429, -0.27305, 0.187325),
        new Rotation3d(0,.1745,0)); 
    //-0.698

    public static final double kFieldLengthMeters = 17.55;
    public static final double kFieldWidthMeters = 8.05;
    // DOCS: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
    // Z-Rotation is Yaw, X-Rotation is Pitch
    // Rotation3d has roll first, then pitch, then yaw
  
    public static final List<AprilTag> kBlueTagList = 
                                        List.of(
                                        new AprilTag(1, new Pose3d(657.37*kInchesToMeters, 25.80*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,126*Math.PI/180.0))),
                                        new AprilTag(2, new Pose3d(657.37*kInchesToMeters, 291.20*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,234*Math.PI/180.0))),
                                        new AprilTag(3, new Pose3d(455.15*kInchesToMeters, 317.15*kInchesToMeters, 51.25*kInchesToMeters, new Rotation3d(0,0,270*Math.PI/180.0))),
                                        new AprilTag(4, new Pose3d(3.6520*kInchesToMeters,241.64*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30*Math.PI/180.0,0))),
                                        new AprilTag(5, new Pose3d(365.20*kInchesToMeters, 75.39*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30*Math.PI/180.0,0))),
                                        new AprilTag(6, new Pose3d(530.49*kInchesToMeters, 130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,300*Math.PI/180.0))),
                                        new AprilTag(7, new Pose3d(546.87*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,0))),
                                        new AprilTag(8, new Pose3d(530.49*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,60*Math.PI/180.0))),
                                        new AprilTag(9, new Pose3d(497.77*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,120*Math.PI/180.0))),
                                        new AprilTag(10, new Pose3d(481.39*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,180*Math.PI/180.0))),
                                        new AprilTag(11, new Pose3d(497.77*kInchesToMeters,130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,240*Math.PI/180.0))),
                                        new AprilTag(12, new Pose3d(33.51*kInchesToMeters, 25.80*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,54*Math.PI/180.0))),
                                        new AprilTag(13, new Pose3d(33.51*kInchesToMeters, 291.20*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,306*Math.PI/180.0))),
                                        new AprilTag(14, new Pose3d(325.68*kInchesToMeters, 241.64*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30*Math.PI/180.0,180*Math.PI/180.0))),
                                        new AprilTag(15, new Pose3d(325.68*kInchesToMeters, 75.39*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30*Math.PI/180.0,180*Math.PI/180.0))),
                                        new AprilTag(16, new Pose3d(235.73*kInchesToMeters, -0.15*kInchesToMeters, 51.25*kInchesToMeters, new Rotation3d(0,0,90*Math.PI/180.0))),
                                        new AprilTag(17, new Pose3d(160.39*kInchesToMeters, 130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,240*Math.PI/180.0))),
                                        new AprilTag(18, new Pose3d(144.00*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,180*Math.PI/180.0))),
                                        new AprilTag(19, new Pose3d(160.39*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,120*Math.PI/180.0))),
                                        new AprilTag(20, new Pose3d(193.10*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,60*Math.PI/180.0))),
                                        new AprilTag(21, new Pose3d(209.49*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,0))),
                                        new AprilTag(22, new Pose3d(193.10*kInchesToMeters, 130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,300*Math.PI/180.0)))
                                    
                                        );                                    

  }

  public static final class RedFieldConstants implements FieldConstants {
    public Pose2d getUpperPickupPos() {
      return  new Pose3d(657.37*kInchesToMeters, 291.20*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,234)).toPose2d();
      //2
    }
    public AprilTag[] getReefTags(){

      return(
      new AprilTag[] {

        VisionConstants.kBlueTagList.get(5), // bc zero index
        VisionConstants.kBlueTagList.get(6), // bc zero index
        VisionConstants.kBlueTagList.get(7), // bc zero index
        VisionConstants.kBlueTagList.get(8), // bc zero index
        VisionConstants.kBlueTagList.get(9), // bc zero index
        VisionConstants.kBlueTagList.get(10), // bc zero index
       

      });
      

    }
    public Pose2d getLowerPickupPos(){

      return new  Pose3d(657.37*kInchesToMeters, 25.80*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,126)).toPose2d();
    }

    public Pose2d getFrontMiddlePlacementPos() {
      return new Pose3d(144.00*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,180*Math.PI/180.0)).toPose2d();
    }
    public List<AprilTag> getAprilTagList(){
      return VisionConstants.kBlueTagList;
    }

    public boolean isOnRed() {return true;}






  }
  public static final class BlueFieldConstants implements FieldConstants {
    public Pose2d getUpperPickupPos(){

      return new Pose3d(33.51*kInchesToMeters, 291.20*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,306)).toPose2d();


    }
    public AprilTag[] getReefTags(){

      return(
      new AprilTag[] {

        VisionConstants.kBlueTagList.get(16), // bc zero index
        VisionConstants.kBlueTagList.get(17), // bc zero index
        VisionConstants.kBlueTagList.get(18), // bc zero index
        VisionConstants.kBlueTagList.get(19), // bc zero index
        VisionConstants.kBlueTagList.get(20), // bc zero index
        VisionConstants.kBlueTagList.get(21), // bc zero index
       

      });
      

    }
    public List<AprilTag> getAprilTagList(){
      return VisionConstants.kBlueTagList;
    }
    public Pose2d getLowerPickupPos(){
      return new Pose3d(33.51*kInchesToMeters, 25.80*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,54)).toPose2d();


    }

    public boolean isOnRed() {return false;}
  }


  public static final class AutoConstants {
    //public static final double kAutoMaxSpeedMetersPerSecond = 4.481;
    //public static final double kAutoMaxAccelerationMetersPerSecondSquared = 11.8;

    public static final double kAutoMaxAngularSpeedRadiansPerSecond =
        1.5 / DriveConstants.kTurningRadiusMeters;
    public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = 36.86135;

    public static final double kPAutoXController = 5; //5, 1, 1, 5, 1, 1
    public static final double kIAutoXController = 1;
    public static final double kDAutoXController = 1;
    public static final double kPAutoYController = 5;
    public static final double kIAutoYController = 1;
    public static final double kDAutoYController = 1;

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

  public static final class PivotConstants{
    public static final double kPivotGearRatio = 27*3; // 60:12 
    //public static final double kPivotSprocketCircumference = 1.432*Math.PI; // INCHES
    //public static final double kPivotInchesToOutput = (kPivotGearRatio)/(kPivotSprocketCircumference);
    public static final int kPivot1Port = 12;



    public static final double kSPivot = 0;
    public static final double kVPivot = 0.35;//0.53;
    public static final double kAPivot = 0.02; // 0.04
    public static final double kGPivot = 0.89;
    public static final double kPPivot = 2;
    public static final double kIPivot = 0;
    public static final double kDPivot = 0;
    public static final double kPivotCruiseVelocity = 20;
    public static final double kPivotAcceleration =64;
    public static final double kPivotJerk = 100;

    public static final boolean kPivotClockwisePositive = true;

    public static final double kPivotErrorTolerance = 5;



  }
  public static final class WristConstants{
    public static final double kWristGearRatio = 36; // reductions 
    //public static final double kPivotSprocketCircumference = 1.432*Math.PI; // INCHES
    //public static final double kPivotInchesToOutput = (kPivotGearRatio)/(kPivotSprocketCircumference);
    public static final int kWrist1Port = 9;



    public static final double kSWrist = 0;
    public static final double kVWrist = 0.53;
    public static final double kAWrist = 0.06;
    public static final double kGWrist = 0;
    public static final double kPWrist = 5;
    public static final double kIWrist = 0;
    public static final double kDWrist = 0;
    public static final double kWristCruiseVelocity = 20;
    public static final double kWristAcceleration = 40;
    public static final double kWristJerk = 80;

    public static final boolean kWristClockwisePositive = true;

    public static final double kWristErrorTolerance = 1;



  }
  public static final class ElevatorConstants{

    public static final double kElevatorGearRatio = 5; // 60:12 
    public static final double kElevatorSprocketCircumference = 1.432*Math.PI; // INCHES
    public static final double kELevatorInchesToOutput = (kElevatorGearRatio)/(kElevatorSprocketCircumference);
    public static final int kElevator1CanID = 10;
    public static final int kElevator2CANID = 11;


    public static final int kElevator1Port = 10; // TODO: placeholder port
    public static final int kElevator2Port = 11;
    public static final double kSElevator = 0;
    public static final double kVElevator = 7.52;
    public static final double kAElevator = 0.01;
    public static final double kGElevator = 0.04;
    public static final double kPElevator = 0.01;
    public static final double kIElevator = 0;
    public static final double kDElevator = 0;
    public static final double kElevatorCruiseVelocity = 0;
    public static final double kElevatorAcceleration = 0;
    public static final double kElevatorJerk = 0;

    public static final boolean kElevatorClockwisePositive = true;

    public static final double kElevatorErrorTolerance = 0.5;

  }

  public static final class RollerConstants{

    
    public static final int kRollerPort = 13; // TODO: placeholder port
    public static final double kPRoller = 0;
    public static final double kIRoller = 0;
    public static final double kDRoller = 0;

    public static final boolean kRollerClockwisePositive = true;

    public static final double kRollerErrorTolerance = 0;
    



  }
  public static final class Swerve 
  {



    public static final int pigeonID = 22;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = 
        COTSFalconSwerveConstants.SDSMK4n(COTSFalconSwerveConstants.driveGearRatios.SDSMK4n_L1);

    /* Drivetrain Constants */
    public static final double lengthBetweenModules = 0.813;
    public static final double widthBetweenModules = 0.686;
    public static final double driveRadius = 0.413091;
    public static final double wheelCircumference = 2*Math.PI * 0.0508;
    
    // public static RobotConfig pathplannerRobotConfig;

    //new RobotConfig(null, null, null, null)
    // public static final HolonomicPathFollowerConfig kPathFollowingConfig = new HolonomicPathFollowerConfig(// HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                     new PIDConstants(2, 0.0, 0), // Translation PID constants
    //                     new PIDConstants(2, 0, 0), // Rotation PID constants
    //                     6, // Max module speed, in m/s
    //                     driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //                     new ReplanningConfig());
    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(new PIDConstants(8,.05,0.4),new PIDConstants(1,0,0));

 


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

    public static final double angleKP = 40;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.05;
    public static final double angleKF = 0;
    public static final double angleKV = 0;
    /* Drive Motor PID Values */
    public static final double driveKP = 0.2;
    public static final double driveKI = 0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0);
    public static final double driveKV = (0.124);
    public static final double driveKA = (0);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.481 ; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 3; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // Tuned to robot
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.01123046875); 
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // Tuned to robot)
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.130033203125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Right Module - Module 2 */
    public static final class Mod2 { // Tuned to robot
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 14;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.08935546875);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Left Module - Module 3- */
    public static final class Mod3 { // Tuned to robot
      public static final int driveMotorID = 6;
      public static final int angleMotorID =5;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.379150390625);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }
  }
}
