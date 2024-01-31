package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    Pose3d savedResult = new Pose3d(0.0,0.0,0.0,new Rotation3d(0.0,0.0,0.0));
    private static PhotonVision pvisioninstance;
    PhotonCamera camera = new PhotonCamera("FrontCam");
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, cameraPitch, 0));
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    Transform3d savedResult3 = new Transform3d();

    private static double cameraHeight = 0.3;
    private static double targetHeight = 1;
    private static double cameraPitch = 0;

    AnalogGyro gyro = new AnalogGyro(0);

    private PhotonVision() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera, robotToCam);
    }

    public static PhotonVision getInstance() {
        if (pvisioninstance == null) {
            pvisioninstance = new PhotonVision();
        }

        return pvisioninstance;
    }
  
  public double getDistance() {

    PhotonTrackedTarget target = getBestTarget(getPipeline());
    if (target == null) return -1;
    Optional<Pose3d> targetPose3dOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    if (targetPose3dOptional.isEmpty()) return -1;
    Pose3d targetPose3d = targetPose3dOptional.get();
    double distance =
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeight,
            targetPose3d.getZ(),
            cameraPitch,
            Units.degreesToRadians(target.getPitch()));
    return distance;
  }

  // todo when implementing with odemoetry make sure not to return savedrselut2 in if-statement
  // compensenate for angle of the camera --> whip out trig

  public Transform3d getRobotPose3dFromTag() {
    PhotonPipelineResult result = getPipeline();
    if (!result.hasTargets()) {
      return savedResult3;
    }
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
      savedResult3 = fieldToCamera;
      return fieldToCamera;
    } else {

      return savedResult3;
    }
    /*
    PhotonTrackedTarget target = getBestTarget(result);
    savedTarget = target;
    Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());

    if (tagPoseOptional.isEmpty()) {
      return savedResult2;
    } else {
      Pose3d tagPose = tagPoseOptional.get();
      Transform3d camToTarget = target.getBestCameraToTarget();
      savedResult2 = tagPose.plus(camToTarget.inverse());
      return savedResult2;
      */
  }


  public Transform3d getTransformToTarget() {
    PhotonPipelineResult pipeline = getPipeline();
    if (!pipeline.hasTargets()) {
      return new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    }
    PhotonTrackedTarget target = getBestTarget(pipeline);
    SmartDashboard.putNumber("pose ambiguity", target.getPoseAmbiguity());
    return target.getBestCameraToTarget();
  }

  public Pose3d getTagPose(int id) {
    Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(id);
    if (tagPoseOptional.isEmpty()) return null;
    return tagPoseOptional.get();
  }

  public double get3dDist() {
    return getTransformToTarget().getTranslation().getNorm();
  }

  public double get3dDist(Transform3d robotPose) {
    double dx = robotPose.getX();
    double dy = robotPose.getY();
    double dz = robotPose.getZ();
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  public void testPose3d() {
    PhotonPipelineResult pipeline = getPipeline();
    PhotonTrackedTarget target = getBestTarget(pipeline);

    TargetCorner c = new TargetCorner(100, 200);

    Pose3d tagPose = getTagPose(target.getFiducialId());
    double d3 = get3dDist();
    double h = tagPose.getZ();
    double zt = target.getYaw();
  }

  /**
   * Gives the Pose3d of the AprilTag with the given ID using the built-in AprilTagFieldLayout.
   * @param tagID ID of the AprilTag to find the Pose3d for
   * @return The Pose3d of the given tag, or null if none is found
   */
  public Pose3d getTagPose3d(int tagID){
    Optional<Pose3d> optional = aprilTagFieldLayout.getTagPose(tagID);
    if(optional.isPresent()) {
      return optional.get();
    } else {
      return null;
    }
  }

  /**
   * Get Robot Pose3d using math (robot yaw and 3d dist).
   * Assuming that passed target parameter is not null.
   */
  public Pose3d getMathRobotPose3d(PhotonTrackedTarget target){
    int tagID = target.getFiducialId();
    Pose3d tagPose3d = getTagPose3d(tagID);
    double visibleAngle = angleFromScreen(target);

    return null;
  }

  /**
   * Gets the angle from camera normal to detected target, in degrees
   */
  public double angleFromScreen(PhotonTrackedTarget target) {
    // PhotonPipelineResult pipeline = getPipeline();
    // PhotonTrackedTarget target = getBestTarget(pipeline);
    // if (!pipeline.hasTargets() || target == null) return Double.NaN;

    // Get tag center pixel
    List<TargetCorner> corners = target.getDetectedCorners();
    double sumX = 0;
    double sumY = 0;
    for (TargetCorner corner : corners) {
      sumX += corner.x;
      sumY += corner.y;
    }

    return 0.0;
  }

    public PhotonPipelineResult getPipeline() {
        return camera.getLatestResult();
    }

    public boolean hasTarget(PhotonPipelineResult pipeline) {
        return pipeline.hasTargets();
    }

    public double getYaw(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    public double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }


    public Pose3d getRobotPose3d() {
        // //TODO: If has no target, the numbers freeze, so make it so that savedResult gets cleared
        // Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        // if (result.isPresent()) {
        //     savedResult = result.get().estimatedPose;
        //     return savedResult;
        // } else {\
        //     return savedResult;
        // }
        PhotonPipelineResult result = getPipeline();
        if(!result.hasTargets()){
            return savedResult;
        }
        
        PhotonTrackedTarget target = getBestTarget(getPipeline());
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        SmartDashboard.putNumber("Tag pose x", tagPose.get().getX());
        SmartDashboard.putNumber("Tag pose y", tagPose.get().getY());
        SmartDashboard.putNumber("Tag pose z", tagPose.get().getZ());
        if(tagPose.isEmpty()){
            return savedResult;
        }
        else{
            savedResult =  PhotonUtils.estimateFieldToRobotAprilTag(getTransformToTarget(),tagPose.get(),robotToCam);
        }
        return savedResult;

    }

    public Pose2d getRobotPose2d() {
        // This tries to use PhotonUtils.estimateFieldToRobot() to get the robot's
        // Pose2d.
        // I followed the PhotonVision documentation:
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html

        // TO-DO: Test this on robot and fix anything that needs fixing for this to
        // work.

        // these shouldn't need tuning.
        double targetPitch = Units.degreesToRadians(getBestTarget(getPipeline()).getPitch());
        Rotation2d targetYaw = Rotation2d.fromDegrees(-getBestTarget(getPipeline()).getYaw());

        // need the pose of the AprilTag. somehow use TargetID to get the pose.
        // int targetID = target.getFiducialId();
        Pose2d targetPose = new Pose2d(-2, 3, new Rotation2d());

        // need to tune this constant later
        Transform2d cameraToRobot = new Transform2d(0.5, 0.5, new Rotation2d());

        // PhotonVision's built in Pose2D estimator. Need to have actual parameters.
        // the gyro.getRotation2d() has to be rechecked; not sure if this is the right
        // way to implement.
        Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
                cameraHeight, targetHeight, cameraPitch, targetPitch, targetYaw, gyro.getRotation2d(), targetPose,
                cameraToRobot);

        // for testing: try displaying the robotPose
        return robotPose;
    }

    public double metersToInches(double meters){
      return 39.3701 * meters;
    }

    public void periodic() {
        PhotonPipelineResult result = getPipeline();
        if(result.hasTargets()){
            double x = getTransformToTarget().getX();
            double z = getTransformToTarget().getZ();
            double y = getTransformToTarget().getY();
            double dist = get3dDist();
            SmartDashboard.putNumber("ID", result.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("PoseX", getRobotPose3d().getX());
            SmartDashboard.putNumber("PoseY", getRobotPose3d().getY());
            SmartDashboard.putNumber("PoseZ", getRobotPose3d().getZ());
            SmartDashboard.putNumber("Angle z from pose", Math.toDegrees(getRobotPose3d().getRotation().getZ()));
            SmartDashboard.putNumber("X",x);
            SmartDashboard.putNumber("Y",y);
            SmartDashboard.putNumber("Y from Distance: ", Math.sqrt(dist * dist - (x*x) - (z*z)));
            SmartDashboard.putNumber("Z", z);
            SmartDashboard.putNumber("Dist", dist);
            // double x=getBestTarget(result).getBestCameraToTarget().getX();
            // double y= getBestTarget(result).getBestCameraToTarget().getY();
            // SmartDashboard.putNumber("Angle",Math.atan(x/y));

        }
        

        


    }

}