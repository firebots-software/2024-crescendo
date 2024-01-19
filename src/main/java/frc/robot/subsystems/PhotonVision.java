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
    private static PhotonVision pvisioninstance;
    PhotonCamera camera = new PhotonCamera("FrontCam");
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, -0.25, 0.25), new Rotation3d(0, 0, 0));
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;

    private static double cameraHeight = 0.3;
    private static double targetHeight = 1;
    private static double cameraPitch = Math.PI / 8;

    PhotonPipelineResult pipeline = getPipeline();
    PhotonTrackedTarget target = bestTarget(pipeline);
    AnalogGyro gyro = new AnalogGyro(0);
    private DoubleLogEntry pitchLog = new DoubleLogEntry(DataLogManager.getLog(), "/log/input/pitch");
    private DoubleLogEntry yawlog = new DoubleLogEntry(DataLogManager.getLog(), "/log/input/yaw");
    private DoubleLogEntry distancelog = new DoubleLogEntry(DataLogManager.getLog(), "/log/input/distance");
    private DoubleLogEntry poselog = new DoubleLogEntry(DataLogManager.getLog(), "/log/input/pose");

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

    public PhotonTrackedTarget bestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public double getDistance() {

        PhotonTrackedTarget target = bestTarget(getPipeline());
        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch,
                Units.degreesToRadians(target.getPitch()));
        return distance;
    }

    public Pose3d getRobotPose3d() {
        //TODO: If has no target, the numbers freeze, so make it so that savedResult gets cleared
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if (result.isPresent()) {
            savedResult = result.get().estimatedPose;
            return savedResult;
        } else {
            return savedResult;
        }

    }

    public Transform3d getTransformToTarget(){
        PhotonPipelineResult pipeline = getPipeline();
        if(!pipeline.hasTargets()){
            return new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        }
        PhotonTrackedTarget target = bestTarget(getPipeline());
        SmartDashboard.putNumber("pose ambiguity", target.getPoseAmbiguity());
        return target.getBestCameraToTarget();
    }

    public double get3dDist(){
        return getTransformToTarget().getTranslation().getNorm();
    }

    public Pose2d getRobotPose2d() {
        // This tries to use PhotonUtils.estimateFieldToRobot() to get the robot's
        // Pose2d.
        // I followed the PhotonVision documentation:
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html

        // TO-DO: Test this on robot and fix anything that needs fixing for this to
        // work.

        // these shouldn't need tuning.
        double targetPitch = Units.degreesToRadians(target.getPitch());
        Rotation2d targetYaw = Rotation2d.fromDegrees(-target.getYaw());

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

        SmartDashboard.putNumber("PoseX", getRobotPose3d().getX());
        SmartDashboard.putNumber("PoseY", getRobotPose3d().getY());
        SmartDashboard.putNumber("PoseZ", getRobotPose3d().getZ());
        SmartDashboard.putNumber("Rot Z", getRobotPose3d().getRotation().getAngle());
        // SmartDashboard.putNumber("Transform X", getTransformToTarget().getTranslation().getX());
        // SmartDashboard.putNumber("Transform Y", getTransformToTarget().getTranslation().getY());
        // SmartDashboard.putNumber("Transform Z", getTransformToTarget().getTranslation().getZ());
        // SmartDashboard.putNumber("Transform Rot Angle", getTransformToTarget().getRotation().getAngle());
        SmartDashboard.putNumber("Dist", get3dDist());

        


    }

}
