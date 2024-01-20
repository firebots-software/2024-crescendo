package frc.robot.subsystems;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.annotation.Target;
import java.sql.Time;
import java.util.List;
import java.util.Optional;

import javax.management.RuntimeErrorException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    Pose3d savedResult = new Pose3d(0.0,0.0,0.0,new Rotation3d(0.0,0.0,0.0));
    Pose3d savedResult2 = new Pose3d(0.0,0.0,0.0,new Rotation3d(0.0,0.0,0.0));
    private static PhotonVision pvisioninstance;
    PhotonCamera camera = new PhotonCamera("FrontCam");
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, cameraPitch, 0));
    PhotonTrackedTarget savedTarget = getBestTarget(getPipeline());
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;

    private static double cameraHeight = 0.004;
    private static double targetHeight = 1; // not used
    private static double cameraPitch = 0.54; // 31 degrees when testing on the floor (1/20/24)

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
        Pose3d targetPose3d = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
        if(target == null || targetPose3d == null) return -1;
        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetPose3d.getZ(), cameraPitch,
                Units.degreesToRadians(target.getPitch()));
        return distance;
    }

    public Pose3d getRobotPose3d() {
        // //TODO: If has no target, the numbers freeze, so make it so that savedResult gets cleared
        // Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        // if (result.isPresent()) {
        //     savedResult = result.get().estimatedPose;
        //     return savedResult;
        // } else {
        //     return savedResult;
        // }
        PhotonPipelineResult result = getPipeline();
        if(!result.hasTargets()){
            return savedResult;
        }
        
        PhotonTrackedTarget target = getBestTarget(getPipeline());
        savedTarget = target;
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

    public Pose3d getRobotPose3dFromTag(){
        PhotonPipelineResult result = getPipeline();
        if(!result.hasTargets()){
            return savedResult2;
        }

        PhotonTrackedTarget target = getBestTarget(getPipeline());
        savedTarget = target;

        Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());

        if(tagPoseOptional.isEmpty()){
            return savedResult2;
        }
        else{
            Pose3d tagPose = tagPoseOptional.get();
            Transform3d camToTarget = getTransformToTarget(target);
            savedResult2 = tagPose.plus(camToTarget.inverse());
            return savedResult2;
        }
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

    public Transform3d getTransformToTarget(PhotonTrackedTarget target){
        //SmartDashboard.putNumber("pose ambiguity", target.getPoseAmbiguity());
        //PhotonUtils.estimateCameraToTargetTranslation(cameraHeight, null);
        //PhotonUtils.estimateCameraToTarget(null, getRobotPose2d(), null)
        return target.getBestCameraToTarget();
    }

    public double get3dDist(){
        return getTransformToTarget().getTranslation().getNorm();
    }

    public double get3dDistFromPose(){
        Pose3d robotPose = getRobotPose3dFromTag();
        Pose3d tagPose = aprilTagFieldLayout.getTagPose(savedTarget.getFiducialId()).get();
        double dx = robotPose.getX()-tagPose.getX();
        double dy = robotPose.getY()-tagPose.getY();
        double dz = robotPose.getZ()-tagPose.getZ();
        double dist = Math.sqrt(dx*dx+dy*dy+dz*dz);
        return dist;
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

    public void periodic() {
        // current problem:
        // getTransformToTarget() is innacurate, specifically the Z coordinate (off by 26 inches).
        // proposal: instead of using getBestCameraToTarget, try getting the 2D transform instead of 3D.
        // (doing this should be more accurate.) then, convert to 3D using the cam height and target height.
        Pose3d robotPose3d = getRobotPose3dFromTag();
        Transform3d transformToTarget = getTransformToTarget();

        SmartDashboard.putNumber("PoseX", robotPose3d.getX());
        SmartDashboard.putNumber("PoseY", robotPose3d.getY());
        SmartDashboard.putNumber("PoseZ", robotPose3d.getZ());
        SmartDashboard.putNumber("Rot Z", robotPose3d.getRotation().getAngle());
        SmartDashboard.putNumber("Transform X", transformToTarget.getTranslation().getX());
        SmartDashboard.putNumber("Transform Y", transformToTarget.getTranslation().getY());
        SmartDashboard.putNumber("Transform Z", transformToTarget.getTranslation().getZ());
        SmartDashboard.putNumber("Transform Rot Angle", transformToTarget.getRotation().getAngle());
        SmartDashboard.putNumber("Dist", get3dDist());
        SmartDashboard.putNumber("Dist2", get3dDistFromPose());
        SmartDashboard.putNumber("Dist3", getDistance());
        SmartDashboard.putNumber("Gyro", gyro.getAngle());

        


    }

}
