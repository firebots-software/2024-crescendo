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
        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch,
                Units.degreesToRadians(target.getPitch()));
        return distance;
    }

    public Pose3d getRobotPose3d() {

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

    public Transform3d getTransformToTarget(){
        PhotonPipelineResult pipeline = getPipeline();
        if(!pipeline.hasTargets()){
            return new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        }
        PhotonTrackedTarget target = getBestTarget(getPipeline());
        SmartDashboard.putNumber("pose ambiguity", target.getPoseAmbiguity());
        return target.getBestCameraToTarget();
    }

    public double get3dDist(){
        return getTransformToTarget().getTranslation().getNorm();
    }

    public Pose2d getRobotPose2d() {
      Pose3d p = getRobotPose3d();
      return new Pose2d(p.getX(), p.getY(), new Rotation2d(p.getRotation().getAngle()));
    }

    public double metersToInches(double meters){
      return 39.3701 * meters;
    }

    public void periodic() {
        PhotonPipelineResult result = getPipeline();
        PhotonTrackedTarget target = getBestTarget(getPipeline());
        if(result.hasTargets()){
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            double x = getTransformToTarget().getX();
            double z = getTransformToTarget().getZ();
            double y = getTransformToTarget().getY();
            double dist = get3dDist();
            SmartDashboard.putNumber("ID", result.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("PoseX", getRobotPose3d().getX());
            SmartDashboard.putNumber("PoseY", getRobotPose3d().getY());
            SmartDashboard.putNumber("PoseZ", getRobotPose3d().getZ());
            SmartDashboard.putNumber("Angle z from pose", Math.toDegrees(getRobotPose3d().getRotation().getZ()));
            SmartDashboard.putNumber("X",tagPose.get().getX() - getRobotPose3d().getX());
            SmartDashboard.putNumber("Y",tagPose.get().getY() - getRobotPose3d().getY());
            SmartDashboard.putNumber("Z", tagPose.get().getZ() - getRobotPose3d().getZ());
            SmartDashboard.putNumber("Dist", dist);
            // double x=getBestTarget(result).getBestCameraToTarget().getX();
            // double y= getBestTarget(result).getBestCameraToTarget().getY();
            // SmartDashboard.putNumber("Angle",Math.atan(x/y));

        }
        

        


    }

}