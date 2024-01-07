package frc.robot.subsystems;

import java.sql.Time;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
   private static PhotonVision pvisioninstance; 
   PhotonCamera camera = new PhotonCamera("Cam");

   private static double cameraHeight = 1;
   private static double targetHeight = 1;
   private static double cameraPitch = 30;
 
   PhotonPipelineResult pipeline = getPipeline();
   PhotonTrackedTarget target =  bestTarget(pipeline);
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
    public void periodic(){
        pitchLog.append(getPitch(target));
        yawlog.append(getYaw(target));
        distancelog.append(getDistance());
    }
  
}