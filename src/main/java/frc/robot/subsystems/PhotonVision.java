package frc.robot.subsystems;

import java.sql.Time;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
   private static PhotonVision pvisioninstance; 
   PhotonCamera camera = new PhotonCamera("Cam");

   private static double cameraHeight = 0.3;
   private static double targetHeight = 1;
   private static double cameraPitch = Math.PI/8;
 
   PhotonPipelineResult pipeline = getPipeline();
   PhotonTrackedTarget target =  bestTarget(pipeline);
   AnalogGyro gyro = new AnalogGyro(0);
   private DoubleLogEntry pitchLog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/pitch");
   private DoubleLogEntry yawlog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/yaw");
   private DoubleLogEntry distancelog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/distance");
   public static PhotonVision getIntance(){
    if (pvisioninstance == null) {
        pvisioninstance = new PhotonVision();
    }

    return pvisioninstance;
   }

   public PhotonPipelineResult getPipeline(){
    return camera.getLatestResult();
   }

   public boolean hasTarget(PhotonPipelineResult pipeline){
     return pipeline.hasTargets();
   }

   public double getYaw(PhotonTrackedTarget target){
    return target.getYaw();
    }

    public double getPitch(PhotonTrackedTarget target){
        return target.getPitch();
    }

    public PhotonTrackedTarget bestTarget(PhotonPipelineResult result){
        return result.getBestTarget();
    }

    public double getDistance(){
        PhotonTrackedTarget target = bestTarget(pipeline);
        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch, Units.degreesToRadians(target.getPitch()));
        return distance;
    }

    public Pose2d getRobotPose2d(){
        // This tries to use PhotonUtils.estimateFieldToRobot() to get the robot's Pose2d.
        // I followed the PhotonVision documentation:
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html

        // TO-DO: Test this on robot and fix anything that needs fixing for this to work.

        // these shouldn't need tuning.
        double targetPitch = Units.degreesToRadians(target.getPitch());
        Rotation2d targetYaw = Rotation2d.fromDegrees(-target.getYaw());

        // need the pose of the AprilTag. somehow use TargetID to get the pose.
        //int targetID = target.getFiducialId();
        Pose2d targetPose = new Pose2d(-2, 3, new Rotation2d());

        // need to tune this constant later
        Transform2d cameraToRobot = new Transform2d(0.5, 0.5, new Rotation2d());

        // PhotonVision's built in Pose2D estimator. Need to have actual parameters.
        // the gyro.getRotation2d() has to be rechecked; not sure if this is the right way to implement.
        Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
            cameraHeight, targetHeight, cameraPitch, targetPitch, targetYaw, gyro.getRotation2d(), targetPose, cameraToRobot
        );

        // for testing: try displaying the robotPose
        return robotPose;
    }
    
    public void periodic(){
        pitchLog.append(getPitch(target));
        yawlog.append(getYaw(target));
        distancelog.append(getDistance());
    }
  
}