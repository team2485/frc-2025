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
import frc.robot.subsystems.Vision.PoseEstimation;
import frc.robot.subsystems.drive.Drivetrain;
import static frc.robot.Constants.Swerve.*;

public class DriveCommandBuilder {

    static GenericEntry angleGetTest;

    public DriveCommandBuilder(PoseEstimation m_poseEstimation, Drivetrain m_drivetrain) {

        AutoBuilder.configure(
        m_poseEstimation::getCurrentPose,
        m_poseEstimation::setCurrentPose,
        m_drivetrain::getChassisSpeeds,
        m_drivetrain::driveAuto, 
        kDriveController,
        m_drivetrain.pathplannerConfig, 
        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue, 
        m_drivetrain);

    }

    
    public static Command driveToPosition(Drivetrain m_drivetrain, PoseEstimation m_poseEstimation, Supplier<Pose2d> fieldEndPos) {
 
    
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = fieldEndPos.get();
        
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                kTeleopMaxSpeedMetersPerSecond, kTeleopMaxAccelerationMetersPerSecondSquared,
                kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);

        // PathFindHolonomic is confirmed functional without collisions avoidance, AutoBuilder must be used to avoid collision
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // Command pathfindingCommand = new PathfindHolonomic(
        //                                 targetPose,
        //                                 constraints,
        //                                 0.0, // Goal end velocity in m/s. Optional
        //                                 m_poseEstimation::getCurrentPose,
        //                                 m_drivetrain::getChassisSpeeds,
        //                                 m_drivetrain::driveAuto,
        //                                 kPathFollowingConfig, // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
        //                                 0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
        //                                 m_drivetrain // Reference to drive subsystem to set requirements
        //                                 );
        // return pathfindingCommand;


        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,   
            0);
        return pathfindingCommand;
    }
    static Pose2d closestSourcePose;
    public static Command alignToSource (Drivetrain m_dDrivetrain, PoseEstimation m_poseEstimation){

        
        var constants = m_poseEstimation.getFieldConstants();
        closestSourcePose=  constants.getLowerPickupPos();

        

        Pose2d topSourcePos = constants.getUpperPickupPos();

        Pose2d lowerSourcePos = constants.getLowerPickupPos();
        double distToLower = m_poseEstimation.getCurrentPose().getTranslation().getDistance(lowerSourcePos.getTranslation());
        double distToUpper = m_poseEstimation.getCurrentPose().getTranslation().getDistance(topSourcePos.getTranslation());
        
        if(distToLower < distToUpper) {
            closestSourcePose = lowerSourcePos;
            
            //Translation2d basePos = closestSourcePose.getTranslation();
            Rotation2d originalRotation = closestSourcePose.getRotation();
            double xMultiplier = originalRotation.getCos();
            double yMultiplier = originalRotation.getSin();
            Translation2d relativeOffset = new Translation2d(xMultiplier, yMultiplier);
            Transform2d converted = new Transform2d(relativeOffset, Rotation2d.kZero); //origRotation
            closestSourcePose = closestSourcePose.transformBy(converted);

        };

        if(distToUpper <= distToLower) {
            closestSourcePose = topSourcePos;
            //Translation2d basePos = closestSourcePose.getTranslation();
            Rotation2d originalRotation = closestSourcePose.getRotation();
            double xMultiplier = originalRotation.getCos();
            double yMultiplier = originalRotation.getSin();
            Translation2d relativeOffset = new Translation2d(xMultiplier, yMultiplier);
            Transform2d converted = new Transform2d(relativeOffset, Rotation2d.kZero); //origRotation
            closestSourcePose = closestSourcePose.transformBy(converted);
        };
        //return closestSourcePose;
        return driveToPosition(m_dDrivetrain, m_poseEstimation, () -> closestSourcePose);
        
    }

}
