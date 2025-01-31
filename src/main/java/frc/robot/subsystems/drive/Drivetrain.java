package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.Swerve.swerveKinematics;

import java.util.function.Supplier;

import org.opencv.objdetect.CascadeClassifier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID, "Drive");
    GenericEntry absoluteGyroPos;
    GenericEntry currentGyroPos;
    GenericEntry allModuleRotPos;
    GenericEntry rot0;
    GenericEntry rot1;
    GenericEntry rot2;
    GenericEntry rot3;

    public RobotConfig pathplannerConfig;

    private double absoluteGyroPosition = 0;
    ///private Supplier<double[]> rotArray =  {0,0,0,0};
    
    public SwerveModule[] mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    public Pigeon2Configuration config = new Pigeon2Configuration();

    private MedianFilter filter = new MedianFilter(5);

    public Drivetrain() {
        //Shuffleboard.getTab("Autos").addString("ChassisSpeedOut", ::toString);

        //gyro.configFactoryDefault();
        rot0 = Shuffleboard.getTab("Swerve").add("rot0", 0).getEntry();
        rot1 = Shuffleboard.getTab("Swerve").add("rot1", 0).getEntry();
        rot2 = Shuffleboard.getTab("Swerve").add("rot2", 0).getEntry();
        rot3 = Shuffleboard.getTab("Swerve").add("rot3", 0).getEntry();


        absoluteGyroPos = Shuffleboard.getTab("Swerve").add("AbsoluteGyroPos", 0).getEntry();
        currentGyroPos = Shuffleboard.getTab("Swerve").add("CurrentGyroPos", 0).getEntry();
       // allModuleRotPos = Shuffleboard.getTab("Swerve").addDoubleArray("All Modules: ",  rotArray);
        gyro.reset();
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            absoluteGyroPosition = 180;
        try{
          pathplannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            
            e.printStackTrace();
        }
        // mSwerveMods = new SwerveModule[] {
        //     new SwerveModule(0, Constants.Swerve.Mod0.constants),
        //     new SwerveModule(1, Constants.Swerve.Mod1.constants),
        //     new SwerveModule(2, Constants.Swerve.Mod2.constants),
        //     new SwerveModule(3, Constants.Swerve.Mod3.constants)
        // };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw().times(-1), getModulePositions());
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Translation2d centerOfRotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    -rotation, 
                                    Rotation2d.fromDegrees(gyro.getYaw().refresh().getValueAsDouble()* -1) // TODO: all gyro values 
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation),
                                centerOfRotation
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        absoluteGyroPos.setDouble(getChassisSpeeds().vyMetersPerSecond);
        currentGyroPos.setDouble(getYaw().times(-1).getDegrees());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    public void driveRobotRelative(ChassisSpeeds speeds){ // DEPRECATED: USE DRIVEAUTO(). THIS METHOD IS ONLY TO BE USED FOR TESTING "straight line" AUTOS!!

        SwerveModuleState[] testStates = swerveKinematics.toSwerveModuleStates(speeds);
            
        rot0.setDouble(testStates[0].angle.getDegrees());
        rot1.setDouble(testStates[1].angle.getDegrees());
        rot2.setDouble(testStates[2].angle.getDegrees());
        rot3.setDouble(testStates[3].angle.getDegrees());
        
        ChassisSpeeds targetSpeeds = speeds; //ChassisSpeeds.discretize(speeds, 0.02);
        
        SwerveModuleState[] targetStates = swerveKinematics.toSwerveModuleStates(targetSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods)
        {

            mod.setDesiredState(targetStates[mod.moduleNumber], true);
        }


    }
    public void driveWithSuppliedRotation(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Rotation2d absoluteRotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    -rotation, 
                                    absoluteRotation
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        absoluteGyroPos.setDouble(absoluteGyroPosition);
        currentGyroPos.setDouble(getYaw().times(-1).getDegrees());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // public void driveAuto(ChassisSpeeds speeds) {
    //     double rot = -rotationOverrideController.calculate(getYawAbsolute().getDegrees() % 180, rotationOverride.getDegrees());
    //     if (speeds.vxMetersPerSecond < .5 && speeds.vyMetersPerSecond < .5)
    //         rot = 0;
    //     speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getYawAbsolute());
    //     driveWithSuppliedRotation(new Translation2d(speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond),rot, true, false, Rotation2d.fromDegrees(getYawAbsolute().getDegrees() % 180));
    // }

    public void driveAuto(ChassisSpeeds speeds) {
        driveWithSuppliedRotation(new Translation2d(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),speeds.omegaRadiansPerSecond, false, true, Rotation2d.fromDegrees(getYawAbsolute().getDegrees() % 180));
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }    


    public double getPitch(){
        //return gyro.getRoll() + 4;
        return gyro.getPitch().refresh().getValueAsDouble();
    }


    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = new SwerveModulePosition(mod.getPosition().distanceMeters, mod.getCanCoder());
        }
        return positions;
    }

    public SwerveModulePosition[] getModulePositionsInverted() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = new SwerveModulePosition(-mod.getPosition().distanceMeters, mod.getCanCoder());
        }
        return positions;
    }
    
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds
        (getModuleStates());
    }

    public void zeroGyro(){
        absoluteGyroPosition += getYaw().times(-1).getDegrees();
        gyro.setYaw(0);
    }

    public void autoGyro(){
        gyro.setYaw(180);
    }

    
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(filter.calculate(360 - gyro.getYaw().refresh().getValueAsDouble())) : Rotation2d.fromDegrees(filter.calculate(gyro.getYaw().refresh().getValueAsDouble()));
    }

    public Rotation2d getYawAbsolute() {
        return Rotation2d.fromDegrees(absoluteGyroPosition).minus(getYaw()).times(-1);
    }

    public Rotation2d getYawMod() {
        return Rotation2d.fromDegrees(getYawAbsolute().getDegrees() % 360);
    }

    public void setCustomYawAbsolute(double yawAbsolute) {
        absoluteGyroPosition = yawAbsolute;
    }

    public void setCustomYaw(double yaw) {
        gyro.setYaw(yaw);
    }

    // public double continiousLoop(double value, double min, double max) {
    //     double mapValue = value / 360; 
    //     // value of 360 is 1 time 
    //     if (value < min)
    //         return max - (min - value);
    //     return value;
    // }

    public void resetToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){

        }

    //     for(SwerveModule mod : mSwerveMods){
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        
    // }
    }