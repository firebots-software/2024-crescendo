package frc.robot.subsystems;

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
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonVision extends SubsystemBase {
  Pose3d savedResult = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  Pose3d savedResult2 = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  Transform3d savedResult3 = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
  private static PhotonVision pvisioninstance;
  PhotonCamera camera = new PhotonCamera("FrontCam");
  Transform3d robotToCam =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, cameraPitch, 0));
  PhotonTrackedTarget savedTarget = getBestTarget(getPipeline());
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEstimator;

  private static double cameraHeight = 0.004;
  private static double targetHeight = 1; // not used
  private static double cameraPitch = 0.54; // 31 degrees when testing on the floor (1/20/24)

  AnalogGyro gyro = new AnalogGyro(0);

  private PhotonVision() {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
  }

  public static PhotonVision getInstance() {
    if (pvisioninstance == null) {
      pvisioninstance = new PhotonVision();
    }

    return pvisioninstance;
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

    // center x and y of tag in pixels, from top-left origin
    double centerX = sumX / corners.size();
    double centerY = sumY / corners.size();
    SmartDashboard.putNumber("Tag ID", target.getFiducialId());
    SmartDashboard.putNumber("Num of Corners", corners.size());

    // new x coordinate, where origin is center of screen
    double tagX = centerX - 640;

    // percent of half-screen x-coordinate
    double percentX = tagX / 640;

    // get scaled 35-degree-triangle leg length
    // sin(35) = 0.5735764
    double triangleLeg = percentX * 0.5735764;

    // get angle using arcsin()
    double finalAngle = Math.toDegrees(Math.asin(triangleLeg));

    return finalAngle;
  }

  /*
  public double get3dDistFromPose() {
    Pose3d robotPose = getRobotPose3dFromTag();
    Pose3d tagPose = aprilTagFieldLayout.getTagPose(savedTarget.getFiducialId()).get();
    double dx = robotPose.getX() - tagPose.getX();
    double dy = robotPose.getY() - tagPose.getY();
    double dz = robotPose.getZ() - tagPose.getZ();
    double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return dist;
  }
  */

  public void periodic() {
    // current problem:
    // getTransformToTarget() is innacurate, specifically the Z coordinate (off by 26 inches).
    // proposal: instead of using getBestCameraToTarget, try getting the 2D transform instead of 3D.
    // (doing this should be more accurate.) then, convert to 3D using the cam height and target
    // height.
    Transform3d robotPose3d = getRobotPose3dFromTag();
    Transform3d transformToTarget = getTransformToTarget();

    PhotonPipelineResult pipeline = getPipeline();
    PhotonTrackedTarget target = getBestTarget(pipeline);

    if (pipeline.hasTargets() && target != null){
      // SmartDashboard.putNumber("Angle to Tag", angleFromScreen(target));
      Pose3d mathPose3d = getMathRobotPose3d(target);
      SmartDashboard.putNumber("Math PoseX", mathPose3d.getX());
      SmartDashboard.putNumber("Math PoseY", mathPose3d.getY());
      SmartDashboard.putNumber("Math PoseZ", mathPose3d.getZ());
      SmartDashboard.putNumber("Math Rot Z", mathPose3d.getRotation().getAngle());
    }

    SmartDashboard.putNumber("PoseX", robotPose3d.getX());
    SmartDashboard.putNumber("PoseY", robotPose3d.getY());
    SmartDashboard.putNumber("PoseZ", robotPose3d.getZ());
    SmartDashboard.putNumber("Rot Z", robotPose3d.getRotation().getAngle());

    SmartDashboard.putNumber("Transform X", transformToTarget.getTranslation().getX());
    SmartDashboard.putNumber("Transform Y", transformToTarget.getTranslation().getY());
    SmartDashboard.putNumber("Transform Z", transformToTarget.getTranslation().getZ());
    SmartDashboard.putNumber("Transform Rot Angle", transformToTarget.getRotation().getAngle());
    SmartDashboard.putNumber("Transform 3d Dist", get3dDist());
    // SmartDashboard.putNumber("Dist2", get3dDistFromPose());
    SmartDashboard.putNumber("Multitag 3d Dist", get3dDist(robotPose3d));
    // SmartDashboard.putNumber("Gyro", gyro.getAngle());

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
    Pose2d robotPose =
        PhotonUtils.estimateFieldToRobot(
            cameraHeight,
            targetHeight,
            cameraPitch,
            targetPitch,
            targetYaw,
            gyro.getRotation2d(),
            targetPose,
            cameraToRobot);

    // for testing: try displaying the robotPose
    return robotPose;
  }
}
