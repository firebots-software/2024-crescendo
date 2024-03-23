package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
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
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  Pose3d savedResult = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  private PhotonCamera camera;
  private static PhotonVision frontCamera = new PhotonVision("FrontCam");
  private PhotonPipelineResult pipeline;
  private PhotonTrackedTarget bestTarget;

  public static AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEstimator;
  Transform3d camToRobot = // robot relative to camera
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-13), 0, Units.inchesToMeters(7.027)),
          new Rotation3d(0, -Units.degreesToRadians(37), Math.PI));

  private PhotonVision(String name) {
    camera = new PhotonCamera(name);
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, camToRobot); // it was PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR before
    pipeline = camera.getLatestResult();
    bestTarget = pipeline.getBestTarget();
  }

  public static PhotonVision getFrontCamera() {
    if (frontCamera == null) {
      frontCamera = new PhotonVision("FrontCam");
    }

    return frontCamera;
  }

  public boolean hasTarget(PhotonPipelineResult pipeline) {
    return pipeline.hasTargets();
  }

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
              getTransformToTarget(), tagPose.get(), camToRobot);
    }
    return savedResult;
  }

  public Optional<EstimatedRobotPose> getMultiTagPose3d(Pose2d previousRobotPose) {
    photonPoseEstimator.setReferencePose(previousRobotPose);
    return photonPoseEstimator.update();
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
    return new Pose2d(p.getX(), p.getY(), new Rotation2d(p.getRotation().getZ()));
  }

  private void log() {
    SmartDashboard.putBoolean("Tag", pipeline.hasTargets());
    if (pipeline.hasTargets()) {
      double dist = get3dDist();
      Pose3d pose3D = getRobotPose3d();
      SmartDashboard.putNumber("TagID", bestTarget.getFiducialId());
      SmartDashboard.putNumber("PoseX", pose3D.getX());
      SmartDashboard.putNumber("PoseY", pose3D.getY());
      SmartDashboard.putNumber("PoseZ", pose3D.getZ());
      SmartDashboard.putNumber("StraightLineDist", dist);
      SignalLogger.writeBoolean("Found tag", true);
    } else {
      SignalLogger.writeBoolean("Found tag", false);
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

  public void periodicSignalLogger(
      Optional<Pose3d> tagPose, Pose3d pose3D, double dist, Transform3d transformToTarget) {
    SignalLogger.writeDouble("Tag pose x", tagPose.get().getX());
    SignalLogger.writeDouble("Tag pose y", tagPose.get().getY());
    SignalLogger.writeDouble("Tag pose z", tagPose.get().getZ());
    SignalLogger.writeDouble("TagID", bestTarget.getFiducialId());
    SignalLogger.writeDouble("PoseX", pose3D.getX());
    SignalLogger.writeDouble("PoseY", pose3D.getY());
    SignalLogger.writeDouble("PoseZ", pose3D.getZ());
    SignalLogger.writeDouble("TranslationX", transformToTarget.getX());
    SignalLogger.writeDouble("TranslationY", transformToTarget.getY());
    SignalLogger.writeDouble("TranslationZ", transformToTarget.getZ());
    SignalLogger.writeDouble("StraightLineDist", dist);
  }
}
