package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  Pose3d savedResult = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  private PhotonCamera camera = new PhotonCamera("FrontCam");
  private PhotonPipelineResult pipeline;
  private PhotonTrackedTarget bestTarget;
  private static PhotonVision pvisioninstance;
  AprilTagFieldLayout aprilTagFieldLayout;
  Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-13), 0, 7.027), new Rotation3d(0, 0.418879, Math.PI));

  private PhotonVision() {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //         camera, robotToCam);
    pipeline = camera.getLatestResult();
    bestTarget = pipeline.getBestTarget();
  }

  public static PhotonVision getInstance() {
    if (pvisioninstance == null) {
      pvisioninstance = new PhotonVision();
    }

    return pvisioninstance;
  }

  public boolean hasTarget(PhotonPipelineResult pipeline) {
    return pipeline.hasTargets();
  }

  // public double getDistance() {
  //     double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight,
  // cameraPitch,
  //             Units.degreesToRadians(bestTarget.getPitch()));
  //     return distance;
  // }

  public Pose3d getRobotPose3d() {
    if (!pipeline.hasTargets()) {
      return savedResult;
    }
    Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
    if (tagPose.isEmpty()) {
      return savedResult;
    } else {
      savedResult =
          PhotonUtils.estimateFieldToRobotAprilTag(
              getTransformToTarget(), tagPose.get(), robotToCam);
    }
    return savedResult;
  }

  public Transform3d getTransformToTarget() {
    if (!pipeline.hasTargets()) {
      return new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    }
    return bestTarget.getBestCameraToTarget();
  }

  public double get3dDist() {
    return getTransformToTarget().getTranslation().getNorm();
  }

  public Pose2d getRobotPose2d() {
    Pose3d p = getRobotPose3d();
    return new Pose2d(p.getX(), p.getY(), new Rotation2d(p.getRotation().getAngle()));
  }

  private double metersToInches(double meters) {
    return 39.3701 * meters;
  }

  private void log() {
    if (pipeline.hasTargets()) {
      Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
      double x = getTransformToTarget().getX();
      double z = getTransformToTarget().getZ();
      double y = getTransformToTarget().getY();
      double dist = get3dDist();
      Pose3d pose3D = getRobotPose3d();
      Transform3d transformToTarget = getTransformToTarget();
      SmartDashboard.putNumber("Tag pose x", tagPose.get().getX());
      SmartDashboard.putNumber("Tag pose y", tagPose.get().getY());
      SmartDashboard.putNumber("Tag pose z", tagPose.get().getZ());
      SmartDashboard.putNumber("TagID", bestTarget.getFiducialId());
      SmartDashboard.putNumber("PoseX", pose3D.getX());
      SmartDashboard.putNumber("PoseY", pose3D.getY());
      SmartDashboard.putNumber("PoseZ", pose3D.getZ());
      SmartDashboard.putNumber("TranslationX", transformToTarget.getX());
      SmartDashboard.putNumber("TranslationY", transformToTarget.getY());
      SmartDashboard.putNumber("TranslationZ", transformToTarget.getZ());
      SmartDashboard.putNumber("StraightLineDist", dist);
    }
  }

  public PhotonPipelineResult getPipeline() {
    return pipeline;
  }

  @Override
  public void periodic() {
    pipeline = camera.getLatestResult();
    bestTarget = pipeline.getBestTarget();
    log();
  }
}
