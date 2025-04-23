// TODO: Port to new pathplanner stuff


package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularAccelerationRadiansPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.DriveConstants.kTeleopMaxSpeedMetersPerSecond;

import java.nio.file.AtomicMoveNotSupportedException;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.opencv.video.FarnebackOpticalFlow;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.fasterxml.jackson.databind.type.ClassKey;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    //public static GenericEntry targetPoseShuffleboard = Shuffleboard.getTab("Autos").add("DesiredPose", Pose2d.kZero ).getEntry();
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
        PathfindingCommand.warmupCommand().schedule();
    }


    public static double constrainRadians(double rads){

        while(rads < -Math.PI){

            rads+=(Math.PI*2);

        }
        while(rads > Math.PI){


            rads-=(Math.PI*2);

        }
        return rads;

    }

    public static Command driveToBarge(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation){

        Pose2d currentPos = m_PoseEstimation.getCurrentPose();
        double bargeDistanceMeters = 0;
        Rotation2d flip = Rotation2d.k180deg;
        if(DriverStation.getAlliance().get() == Alliance.Blue){

            bargeDistanceMeters = 310.5*Constants.kInchesToMeters;
            

        }else{
            bargeDistanceMeters = Constants.VisionConstants.kFieldLengthMeters - (310.5*Constants.kInchesToMeters);

            flip = Rotation2d.kZero;
        }
        Pose2d targetPose = new Pose2d(bargeDistanceMeters,m_PoseEstimation.getCurrentPose().getY(),flip);
        double dist =  m_PoseEstimation.getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());
        if(dist<.5){

            return driveToPosition(m_Drivetrain, m_PoseEstimation,()->targetPose);

        }
        else{

            PathConstraints constraints = new PathConstraints(3.7, 3, 1.3, 1.3);
            var cmd = AutoBuilder.pathfindToPose(targetPose, constraints);
            return cmd;
            

        }

    }
    public static Command driveToShoot(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation){


        Pose2d target = m_PoseEstimation.getFieldConstants().getBargePose();    

        Rotation2d targetAngle = m_PoseEstimation.getCurrentPose().getTranslation().minus(target.getTranslation()).getAngle() ;
        double constTarget = DriveCommandBuilder.constrainRadians(targetAngle.getRadians());
        double constActual = DriveCommandBuilder.constrainRadians(m_PoseEstimation.getCurrentPose().getRotation().getRadians());



        Pose2d currentPos = m_PoseEstimation.getCurrentPose();
        double bargeDistanceMeters = 0;
        Rotation2d flip = Rotation2d.k180deg;
        if(DriverStation.getAlliance().get() == Alliance.Blue){

            bargeDistanceMeters = 127*Constants.kInchesToMeters; // 107
            flip = Rotation2d.kZero;
            

        }else{
            bargeDistanceMeters = 162*Constants.kInchesToMeters; // 142

            flip = Rotation2d.kZero;

        }
        Translation2d dir = currentPos.minus(PoseEstimation.getFieldConstants().getBargePose()).getTranslation();
        
        Translation2d unit = dir.div(Math.sqrt((dir.getX()*dir.getX()) + (dir.getY()*dir.getY()) ));
        Translation2d actualPose = PoseEstimation.getFieldConstants().getBargePose().getTranslation() .minus(unit.times(bargeDistanceMeters) );
        Pose2d targetPose = new Pose2d(actualPose,new Rotation2d(constTarget + flip.getRadians()));
        double dist =  m_PoseEstimation.getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());
        
        // return driveToPosition(m_Drivetrain, m_PoseEstimation,()->targetPose);
        if(dist <.2 ){

            return null;

        }
        if(dist<.6){

            return driveToPosition(m_Drivetrain, m_PoseEstimation,()->targetPose);

        }
        else{

            PathConstraints constraints = new PathConstraints(6.5, 4, 3, 2.3);
            var cmd = AutoBuilder.pathfindToPose(targetPose, constraints);
            return cmd;
            

        }

    }
    public static Command driveToPosition(Drivetrain m_drivetrain, PoseEstimation m_poseEstimation, Supplier<Pose2d> fieldEndPos) {
 
    
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = fieldEndPos.get();

        double dist =  m_poseEstimation.getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());
        if(dist < .5){ // TODO: revert tolerance to .5
            // PathConstraints constraints = new PathConstraints(1, 1, 0.5,0.5);
            Command shortCommand = shortDriveToPoseMid(m_drivetrain, m_poseEstimation, targetPose);
            //Command shortCommand = shortDriveToPose(m_drivetrain, m_poseEstimation, targetPose);
            return shortCommand;

        }
        else{

            Command shortCommand = shortDriveToPoseFast(m_drivetrain, m_poseEstimation, targetPose);
            return shortCommand;

        }
        
        // Create the constraints to use while pathfinding
        // PathConstraints constraintsOld = new PathConstraints(
        //         kTeleopMaxSpeedMetersPerSecond, kTeleopMaxAccelerationMetersPerSecondSquared,
        // //         kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationRadiansPerSecondSquared);
        // PathConstraints constraints = new PathConstraints(3, 3, 0.5,0.5);

    //      PathConstraints constraints = new PathConstraints(3, 3, 0.5,0.5);
    //    // endPos = new Pose2d(6.35,2.54, Rotation2d.kZero);
    //         var difference =targetPose.minus( m_poseEstimation.getCurrentPose() ) ;
    //         var direction = difference.getRotation();
    //         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //             new Pose2d(m_poseEstimation.getCurrentPose().getTranslation(),Rotation2d.kZero),
    //             new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(0))
    //         // new Pose2d(0.75, 0.25, Rotation2d.fromDegrees(0)),
    //             //new Pose2d(5.0*0.25, 3.0*0.25, Rotation2d.fromDegrees(90))
            
            
    //     /// new Pose2d(m_PoseEstimation.getCurrentPose().getTranslation(),direction),new Pose2d(6.35,2.54, Rotation2d.kZero)
            
            
    //         );//Pose2d.kZero, new Pose2d(1, 0, Rotation2d.kZero));     //m_PoseEstimation.getCurrentPose(), endPos
    //         PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));
    //         path.preventFlipping = true;
    //         return AutoBuilder.followPath(path);

        
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


        // Command pathfindingCommand = AutoBuilder.pathfindToPose(
        //     targetPose,
        //     constraints,   
        //     0);
        // return pathfindingCommand;
    }

    public static int findNearestScoringTagId(PoseEstimation m_poseEstimation){

        var constants = m_poseEstimation.getFieldConstants();
        AprilTag[] tags = constants.getReefTags();
        AprilTag nearestTag = tags[0];
        double closestDistance = Double.MAX_VALUE;
        Pose2d currestPose = m_poseEstimation.getCurrentPose();
        for(AprilTag tag : tags){

            Pose2d tagPose = tag.pose.toPose2d();
            double thisDist = currestPose.getTranslation().getDistance(tagPose.getTranslation());
            if(thisDist < closestDistance){
                closestDistance=thisDist;
                nearestTag = tag;

            }

        }
        return nearestTag.ID;

        
    }

    public static Command shortDriveToPose(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation, Pose2d endPos, PathConstraints constraints){
        // PathPlannerPath.clearCache();
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(m_PoseEstimation.getCurrentPose().getTranslation(),endPos.getRotation() ),
            new Pose2d(endPos.getTranslation(), endPos.getRotation())
        );
        PathPlannerPath path;
        if(DriverStation.isAutonomous()){

            path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(1, endPos.getRotation()));


        }
        else{
        path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, endPos.getRotation()));

        }
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);

    }
    public static Command shortDriveToCoralStation(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation, Pose2d endPos){
        // PathPlannerPath.clearCache();
        PathConstraints constraints = new PathConstraints(12,8.5, 4, 4);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(m_PoseEstimation.getCurrentPose().getTranslation(),endPos.getRotation().rotateBy(Rotation2d.k180deg)),
            new Pose2d(endPos.getTranslation(), endPos.getRotation().rotateBy(Rotation2d.k180deg))
        );
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(1, endPos.getRotation()));
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);

    }
    public static Command shortDriveToPose3Waypoint(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation, Pose2d point1, Pose2d endPos, PathConstraints constraints)
    {   


        // PathPlannerPath.clearCache();
        // PathConstraints constraints = new PathConstraints(7,4.5, 4, 4);
        Rotation2d heading =point1.getTranslation().minus( m_PoseEstimation.getCurrentPose().getTranslation()).getAngle();
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(m_PoseEstimation.getCurrentPose().getTranslation(),m_PoseEstimation.getCurrentPose().getRotation()),
            // new Pose2d(point1.getTranslation(), heading),

            new Pose2d(endPos.getTranslation(), endPos.getRotation().rotateBy(Rotation2d.k180deg))
        );
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, endPos.getRotation()));
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);



    }
    public static Command shortDriveToPoseSlow(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation, Pose2d endPos){

        PathConstraints constraints = new PathConstraints(1, 0.5, 0.5, 0.5);//new PathConstraints(1, 1, 0.5,0.5);
       // endPos = new Pose2d(6.35,2.54, Rotation2d.kZero);
        return shortDriveToPose(m_Drivetrain, m_PoseEstimation, endPos, constraints);

    }
    public static Command shortDriveToPoseMid(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation, Pose2d endPos){

        // endPos = new Pose2d(6.35,2.54, Rotation2d.kZero);
        PathConstraints constraints = new PathConstraints(3.7,3, 1,1);
        // PathConstraints constraints = new PathConstraints(0.5, 0.5, 0.5, 0.5);//new PathConstraints(1, 1, 0.5,0.5);
        
        return shortDriveToPose(m_Drivetrain, m_PoseEstimation, endPos, constraints);
 
     }

    public static Command shortDriveToPoseFast(Drivetrain m_Drivetrain, PoseEstimation m_PoseEstimation, Pose2d endPos){

       // endPos = new Pose2d(6.35,2.54, Rotation2d.kZero);
    //    PathConstraints constraints = new PathConstraints(0.5, 0.5, 0.5, 0.5);//new PathConstraints(1, 1, 0.5,0.5);
       
       PathConstraints constraints = new PathConstraints(3.7, 3, 1.3,1.3);
       return shortDriveToPose(m_Drivetrain, m_PoseEstimation, endPos, constraints);
        
       /// new Pose2d(m_PoseEstimation.getCurrentPose().getTranslation(),direction),new Pose2d(6.35,2.54, Rotation2d.kZero)
        
        
      

    }
    public static Pose2d convertAprilTag(int tagId, double forwardOffset, double sideOffset,Drivetrain m_Drivetrain, PoseEstimation m_poseEstimation, boolean flip ){

        var constants = m_poseEstimation.getFieldConstants();
        List<AprilTag> aprilTagPositions = constants.getAprilTagList();
        var desiredTag = aprilTagPositions.get(tagId-1);


        Pose2d desiredPos = desiredTag.pose.toPose2d();

        // Rotation2d originalRotation = desiredPos.getRotation(); // should be 0.0deg for 21
        // double xMultiplier = originalRotation.getCos();
        // double yMultiplier = originalRotation.getSin();
        // Translation2d relativeOffset = new Translation2d(yMultiplier,-xMultiplier ); // make vector of direction
        if(!flip){

            Transform2d converted = new Transform2d(forwardOffset,sideOffset, Rotation2d.k180deg);
            desiredPos = desiredPos.transformBy(converted);
            return desiredPos;

        }else{

            Transform2d converted = new Transform2d(forwardOffset,sideOffset, Rotation2d.kZero);
            desiredPos = desiredPos.transformBy(converted);
            return desiredPos;
        }
  
        
    }
    public static Pose2d convertAprilTag(int tagId, double forwardOffset, double sideOffset,Drivetrain m_Drivetrain, PoseEstimation m_poseEstimation ){
        return convertAprilTag(tagId, forwardOffset, sideOffset, m_Drivetrain, m_poseEstimation, false);

    }
    public static Command roughAlignToTag(int tagId, double forwardOffset, double sideOffset, Drivetrain m_Drivetrain, PoseEstimation m_poseEstimation){

        return roughAlignToTag(tagId, forwardOffset, sideOffset, m_Drivetrain, m_poseEstimation,false);

    }

    public static Command roughAlignToTag(int tagId, double forwardOffset, double sideOffset, Drivetrain m_Drivetrain, PoseEstimation m_poseEstimation,boolean shouldFlip){

       
        final Pose2d endPos = convertAprilTag(tagId, forwardOffset, sideOffset,m_Drivetrain,m_poseEstimation,shouldFlip);
        //return endPos;
       // targetPoseShuffleboard.setValue(endPos);
        return driveToPosition(m_Drivetrain, m_poseEstimation, () -> endPos);


    }


    public static int findNearestSourceId(PoseEstimation m_poseEstimation, Drivetrain m_Drivetrain){

        var constants = m_poseEstimation.getFieldConstants();
        Pose2d lowerSourcePos = convertAprilTag(constants.getLowerPickupId(),0.0,0.0,m_Drivetrain,m_poseEstimation);
        Pose2d upperSourcePos = convertAprilTag(constants.getUpperPickupId(),0.0,0.0,m_Drivetrain,m_poseEstimation);
        
        //Pose2d upperSourcePos = convertAprilTag(constants.getUpperPickupId(),0,0,0,m_Drivetrain,m_poseEstimation);
        double lowDist = m_poseEstimation.getCurrentPose().getTranslation().getDistance(lowerSourcePos.getTranslation());
        double highDist = m_poseEstimation.getCurrentPose().getTranslation().getDistance(upperSourcePos.getTranslation());
        


        if(lowDist<highDist) return constants.getLowerPickupId();
        else return constants.getUpperPickupId();
        
    }

    // public static Command alignToSource (Drivetrain m_dDrivetrain, PoseEstimation m_poseEstimation){

        
    //     var constants = m_poseEstimation.getFieldConstants();
    //     closestSourcePose=  constants.getLowerPickupPos();

        

    //     Pose2d topSourcePos = constants.getUpperPickupPos();

    //     Pose2d lowerSourcePos = constants.getLowerPickupPos();
    //     double distToLower = m_poseEstimation.getCurrentPose().getTranslation().getDistance(lowerSourcePos.getTranslation());
    //     double distToUpper = m_poseEstimation.getCurrentPose().getTranslation().getDistance(topSourcePos.getTranslation());
        
    //     if(distToLower < distToUpper) {
    //         closestSourcePose = lowerSourcePos;
            
    //         //Translation2d basePos = closestSourcePose.getTranslation();
    //         Rotation2d originalRotation = closestSourcePose.getRotation();
    //         double xMultiplier = originalRotation.getCos();
    //         double yMultiplier = originalRotation.getSin();
    //         Translation2d relativeOffset = new Translation2d(xMultiplier, yMultiplier);
    //         Transform2d converted = new Transform2d(relativeOffset, Rotation2d.kZero); //origRotation
    //         closestSourcePose = closestSourcePose.transformBy(converted);

    //     };

    //     if(distToUpper <= distToLower) {
    //         closestSourcePose = topSourcePos;
    //         //Translation2d basePos = closestSourcePose.getTranslation();
    //         Rotation2d originalRotation = closestSourcePose.getRotation();
    //         double xMultiplier = originalRotation.getCos();
    //         double yMultiplier = originalRotation.getSin();
    //         Translation2d relativeOffset = new Translation2d(xMultiplier, yMultiplier);
    //         Transform2d converted = new Transform2d(relativeOffset, Rotation2d.kZero); //origRotation
    //         closestSourcePose = closestSourcePose.transformBy(converted);
    //     };
    //     //return closestSourcePose;
    //     return driveToPosition(m_dDrivetrain, m_poseEstimation, () -> closestSourcePose);
        
    // }

}
