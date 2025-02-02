package frc.robot.subsystems.Vision;


import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.BlueFieldConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RedFieldConstants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommandBuilder;
import frc.robot.subsystems.drive.Drivetrain;

public class PoseEstimation extends SubsystemBase {
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Supplier<Rotation2d> rotation;
  private final Supplier<SwerveModulePosition[]> modulePosition;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDrivePoseEstimator noVisionPoseEstimator;
  private final Field2d field2d = new Field2d();
 private final Vision frontLeftPhoton = new Vision("cameraFrontLeft"); // TODO: Replace Camera Name here!
 private final Vision frontRightPhoton = new Vision("cameraFrontRight"); // TODO: See if this would work for multicamera localization.
  private final Notifier photonNotifier = new Notifier(frontLeftPhoton);
 
  private final Notifier photonNotifier2 = new Notifier(frontRightPhoton);
  private final WL_CommandXboxController m_driver;
    private final WL_CommandXboxController m_operator;

 // private OriginPosition originPosition = OriginPosition.kRedAllianceWallRightSide;
  private boolean sawTag = false;
  private double angleToTags = 0;
  Supplier<ChassisSpeeds> speeds;


  Drivetrain m_drivetrain;

  GenericEntry visionTest;
  GenericEntry xSped;
  GenericEntry xLog;
  GenericEntry attemptedNavPosition;
  boolean isOnRed;

  public PoseEstimation(Supplier<Rotation2d> rotation, Supplier<SwerveModulePosition[]> modulePosition, Supplier<ChassisSpeeds> chassisSpeeds, WL_CommandXboxController m_driver, WL_CommandXboxController m_operator, Drivetrain m_drivetrain) {
    visionTest = Shuffleboard.getTab("Swerve").add("YSped", 10).getEntry();
    xSped = Shuffleboard.getTab("Swerve").add("XSped", 10).getEntry();
    xLog = Shuffleboard.getTab("Swerve").add("YDist", 0).getEntry();
    attemptedNavPosition = Shuffleboard.getTab("Swerve").add("TargetPos", getFormattedPose(Pose2d.kZero)).getEntry();
    this.rotation = rotation;
    this.modulePosition = modulePosition;
    this.speeds = chassisSpeeds;
    this.m_driver = m_driver;
    this.m_operator = m_operator;

    poseEstimator = new SwerveDrivePoseEstimator(
        Swerve.swerveKinematics,
        rotation.get(),
        modulePosition.get(),
        new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
    

    noVisionPoseEstimator = new SwerveDrivePoseEstimator(
        Swerve.swerveKinematics, 
        rotation.get(),
        modulePosition.get(), 
        new Pose2d());

    this.m_drivetrain = m_drivetrain;

    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);
    photonNotifier2.setName("PhotonRunnable");
    photonNotifier2.startPeriodic(0.02);

    isOnRed = getFieldConstants().isOnRed();
  }
  public void test_set(){

    this.setCurrentPose(new Pose2d(7.169, 2.664, Rotation2d.fromDegrees(-171)));

  }
  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d);//.withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFormattedPose).withPosition(6, 2).withSize(2, 1);
  }

  @Override
  public void periodic() {
    poseEstimator.update(rotation.get(), modulePosition.get());
    noVisionPoseEstimator.update(rotation.get(), modulePosition.get());
    // TODO: For loop over cameras here
    //attemptedNavPosition.setValue(getFormattedPose(DriveCommandBuilder.alignToSource(m_drivetrain, this)));
    var visionPoseFrontLeft = frontLeftPhoton.grabLatestEstimatedPose();
    var visionPoseFrontRight = frontRightPhoton.grabLatestEstimatedPose();
    if(visionPoseFrontLeft != null){

      var pose2d = visionPoseFrontLeft.estimatedPose.toPose2d();
      poseEstimator.addVisionMeasurement(pose2d, visionPoseFrontLeft.timestampSeconds);


    }
    if(visionPoseFrontRight != null){

      var pose2d = visionPoseFrontRight.estimatedPose.toPose2d();
      poseEstimator.addVisionMeasurement(pose2d, visionPoseFrontRight.timestampSeconds);
     
    }
    // var visionPose = photonEstimator.grabLatestEstimatedPose();
    // var theoreticalOtherCamPose = multiCamTest.grabLatestEstimatedPose();
    // if (visionPose != null) { // Multicamera Reference : https://www.chiefdelphi.com/t/multi-camera-setup-and-photonvisions-pose-estimator-seeking-advice/431154/4
    //   var pose2d = visionPose.estimatedPose.toPose2d();
   
    //   poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
      
    // }
    // if(theoreticalOtherCamPose != null){

    //   var pose2d = theoreticalOtherCamPose.estimatedPose.toPose2d();
    //   poseEstimator.addVisionMeasurement(pose2d, theoreticalOtherCamPose.timestampSeconds);

    // }

    // if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
    //   dashboardPose = flipAlliance(dashboardPose);
    // }
      
   // Shuffleboard.getTab("Autos").add(field2d);
    angleToTags = getCurrentPose().getRotation().getDegrees();
    visionTest.setDouble(speeds.get().vyMetersPerSecond);
    xSped.setDouble((isOnRed?-1:1));
    
    //if (getNoteDetected()) m_operator.setRumble(RumbleType.kLeftRumble, 1);
    //else m_operator.setRumble(RumbleType.kLeftRumble, 0);
    
    var dashboardPose = getCurrentPose();
      
    field2d.setRobotPose(dashboardPose);

   // if (getNoteDetected()) m_driver.setRumble(RumbleType.kLeftRumble, 1);
    //else m_driver.setRumble(RumbleType.kLeftRumble, 0);
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f radians", pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }
  private String getFormattedPose(Pose2d toConv) {
    var pose = toConv;
    return String.format("(%.3f, %.3f) %.2f radians", pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }
  public Pose2d getCurrentVisionlessPose() {
    var pos = noVisionPoseEstimator.getEstimatedPosition();
    return pos;
  }

  public Pose2d getCurrentPose() {
    var pos = poseEstimator.getEstimatedPosition();
    
    if (pos.getX() < 0)
      pos = new Pose2d(new Translation2d(0, poseEstimator.getEstimatedPosition().getY()), poseEstimator.getEstimatedPosition().getRotation());
    if (pos.getX() > VisionConstants.kFieldLengthMeters)
      pos = new Pose2d(new Translation2d(VisionConstants.kFieldLengthMeters, poseEstimator.getEstimatedPosition().getY()), poseEstimator.getEstimatedPosition().getRotation());
    
    return pos;
  }

  public Pose2d getCurrentPoseNoVision() {
    return noVisionPoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotation.get(), modulePosition.get(), newPose);
    noVisionPoseEstimator.resetPosition(rotation.get(), modulePosition.get(), newPose);;
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public double dist(Pose2d pos1, Pose2d pos2) {
    double xDiff = pos2.getX()-pos1.getX();
    double yDiff = pos2.getY()-pos1.getY();
    return Math.sqrt(xDiff*xDiff + yDiff*yDiff);
  }

  public double getAngleFromTags() {
    if (angleToTags < -90) return 180 + angleToTags;
    if (angleToTags > 90) return -180 + angleToTags;
    return angleToTags;
  }


  public double getAngleToPos(Pose2d pos) {
    double deltaY = pos.getY() - getCurrentPose().getY();
    double deltaX = pos.getX() - getCurrentPose().getX();
    return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
  }


  

 

  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public FieldConstants getFieldConstants() {
    RedFieldConstants redFieldConstants = new RedFieldConstants();
    BlueFieldConstants blueFieldConstants = new BlueFieldConstants();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return blueFieldConstants;
    }
    else {
      return redFieldConstants;
    }
  }
}