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

  public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
  COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.05;
    public static final double kDriverLeftXDeadband = 0.05;
    public static final double kDriverLeftYDeadband = 0.05;

    public static final double kTriggerThreshold = 0.1;
  }

  public interface FieldConstants {

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
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(.2921, 0, 0.24),
        new Rotation3d(0,-.402,0)); 
    //-0.698

    public static final double kFieldLengthMeters = 17.55;
    public static final double kFieldWidthMeters = 8.05;
    // DOCS: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
    // Z-Rotation is Yaw, X-Rotation is Pitch
    // Rotation3d has roll first, then pitch, then yaw
  
    public static final List<AprilTag> kBlueTagList = 
                                        List.of(
                                        new AprilTag(1, new Pose3d(657.37*kInchesToMeters, 25.80*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,126))),
                                        new AprilTag(2, new Pose3d(657.37*kInchesToMeters, 291.20*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,234))),
                                        new AprilTag(3, new Pose3d(455.15*kInchesToMeters, 317.15*kInchesToMeters, 51.25*kInchesToMeters, new Rotation3d(0,0,270))),
                                        new AprilTag(4, new Pose3d(3.6520*kInchesToMeters,241.64*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30,0))),
                                        new AprilTag(5, new Pose3d(365.20*kInchesToMeters, 75.39*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30,0))),
                                        new AprilTag(6, new Pose3d(530.49*kInchesToMeters, 130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,300))),
                                        new AprilTag(7, new Pose3d(546.87*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,0))),
                                        new AprilTag(8, new Pose3d(530.49*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,60))),
                                        new AprilTag(9, new Pose3d(497.77*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,120))),
                                        new AprilTag(10, new Pose3d(481.39*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,180))),
                                        new AprilTag(11, new Pose3d( 497.77*kInchesToMeters,130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,240))),
                                        new AprilTag(12, new Pose3d(33.51*kInchesToMeters, 25.80*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,54))),
                                        new AprilTag(13, new Pose3d(33.51*kInchesToMeters, 291.20*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,306))),
                                        new AprilTag(14, new Pose3d(325.68*kInchesToMeters, 241.64*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30,180))),
                                        new AprilTag(15, new Pose3d(325.68*kInchesToMeters, 75.39*kInchesToMeters, 73.54*kInchesToMeters, new Rotation3d(0,30,180))),
                                        new AprilTag(16, new Pose3d(235.73*kInchesToMeters, -0.15*kInchesToMeters, 51.25*kInchesToMeters, new Rotation3d(0,0,90))),
                                        new AprilTag(17, new Pose3d(160.39*kInchesToMeters, 130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,240))),
                                        new AprilTag(18, new Pose3d(144.00*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,180))),
                                        new AprilTag(19, new Pose3d(160.39*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,120))),
                                        new AprilTag(20, new Pose3d(193.10*kInchesToMeters, 186.83*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,60))),
                                        new AprilTag(21, new Pose3d(209.49*kInchesToMeters, 158.50*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,0))),
                                        new AprilTag(22, new Pose3d(193.10*kInchesToMeters, 130.17*kInchesToMeters, 12.13*kInchesToMeters, new Rotation3d(0,0,300)))
                                    
                                        );                                    

  }

  public static final class RedFieldConstants implements FieldConstants {
    public Pose2d getUpperPickupPos() {
      return  new Pose3d(657.37*kInchesToMeters, 291.20*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,234)).toPose2d();
      //2
    }
    public Pose2d getLowerPickupPos(){

      return new  Pose3d(657.37*kInchesToMeters, 25.80*kInchesToMeters, 058.50*kInchesToMeters, new Rotation3d(0,0,126)).toPose2d();
    }
    public boolean isOnRed() {return true;}
  }
  public static final class BlueFieldConstants implements FieldConstants {
    public Pose2d getUpperPickupPos(){

      return new Pose3d(33.51*kInchesToMeters, 291.20*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,306)).toPose2d();


    }
    public Pose2d getLowerPickupPos(){
      return new Pose3d(33.51*kInchesToMeters, 25.80*kInchesToMeters, 58.50*kInchesToMeters, new Rotation3d(0,0,54)).toPose2d();


    }

    public boolean isOnRed() {return false;}
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

  public static final class TelescopeConstants{

    
    public static final int kTelescopePort = 0; // TODO: placeholder port
    public static final double kSTelescope = 0;
    public static final double kVTelescope = 0;
    public static final double kPTelescope = 0;
    public static final double kITelescope = 0;
    public static final double kDTelescope = 0;
    public static final double kTelescopeCruiseVelocity = 0;
    public static final double kTelescopeAcceleration = 0;
    public static final double kTelescopeJerk = 0;

    public static final boolean kTelescopeClockwisePositive = true;

    public static final double kTelescopeErrorTolerance = 0;
    



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
    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(new PIDConstants(2,0,0),new PIDConstants(2,0,0));

 


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
