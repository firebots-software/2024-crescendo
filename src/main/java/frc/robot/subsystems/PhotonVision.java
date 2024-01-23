package frc.robot.subsystems;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
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
    // Note: To connect to PhotonVision dashboard, type in http://photonvision.local:5800/ in browser
   // Note: Vision progress doc: https://docs.google.com/document/d/1dcJlagq1mkPYdsJ_vEmH3t5qBNc2CaBrx94pEgkzsrA/edit?usp=sharing
   
   private static PhotonVision instance; 
   private PhotonCamera camera = new PhotonCamera("FrontCam");
   private AprilTagFieldLayout aprilTagFieldLayout;
   private PhotonPoseEstimator photonPoseEstimator;

   private Pose3d savedPoseFromPPE;
   private Pose3d savedPoseFromTag;
   private PhotonPipelineResult savedResult;
   private PhotonTrackedTarget savedTarget;

   private static double cameraHeight = 0.3; // NEEDS TUNING
   private static double targetHeight = 1; // NEEDS TUNING
   private static double cameraPitch = Math.PI/8; // NEEDS TUNING
   // NEEDS TUNING
   private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, -0.25, cameraHeight), new Rotation3d(0, cameraPitch, 0));

    /**
     * PhotonVision constructor (private).
     * @apiNote Initializes AprilTagFieldLayout
     * @apiNote Initializes PhotonPoseEstimator
     * @apiNote Runs camera.getLatestResult()
     * @apiNote Updates target (savedResult.getBestTarget())
     */
   private PhotonVision(){
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
    savedResult = camera.getLatestResult();
    savedTarget = savedResult.getBestTarget();
   }

   /**
    * Checks if there's already an instance of this subsystem. If there is, returns the instance.
    * If not, runs the constructor and stores in "instance".
    * @return The singleton instance of this subsystem (PhotonVision)
    */
   public static PhotonVision getInstance(){
        if (instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    /**
     * Updates the camera pipeline and the best target variable. Target not guaranteed to exist; might be null.
     */
    public void update(){
        savedResult = camera.getLatestResult();
        savedTarget = savedResult.getBestTarget();
    }

    /**
     * (NEED TESTING) Updates the camera pipeline and the target variable, trying to set the target variable to the target that matches
     * the given tagID. If no matching detected targets found, simply sets to getBestTarget().
     * If no targets found, returns null.
     * @param tagID The ID of the AprilTag to look for while detecting.
     */
    public void update(int tagID){
        savedResult = camera.getLatestResult();
        List<PhotonTrackedTarget> targetsFound = savedResult.getTargets();
        PhotonTrackedTarget match = null;
        for (int i = 0; i < targetsFound.size(); i++) {
            PhotonTrackedTarget t = targetsFound.get(i);
            if(t.getFiducialId() == tagID){
                match = t;
                break;
            }
        }
        if(targetsFound.size() > 0 && match == null) match = savedResult.getBestTarget();
        savedTarget = match;
    }

    /**
     * Updates target if desired. Checks to see if targets from the latest result exist.
     * @param runUpdate Whether or not to update target before checking if targets exist
     * @return Whether or not targets exist
     */
    public boolean hasTargets(boolean runUpdate){
        if(runUpdate) update();
        boolean hasTargets = savedResult.hasTargets();
        return hasTargets;
    }

    /**
     * (NEEDS TESTING) Updates target if desired. Checks to see if the latest detected target matches IDs with the given tag ID.
     * @param tagID - The tagID to compare with
     * @return Whether or not the latest target ID matches the given tagID.
     */
    public boolean matchesTagID(int tagID, boolean updateTarget){
        int current = getTagID(tagID, updateTarget);
        return current == tagID;
    }

    /**
     * (NEEDS TESTING) Updates target if desired. Gets the TagID of the latest detected target. Returns 0 if no targets found.
     * @return The tagID of the latest detected target (or 0 if no targets found).
     */
    public int getTagID(boolean updateTarget){
        if(!hasTargets(updateTarget)) return 0;

        return savedTarget.getFiducialId();
    }

    /**
     * Updates target if desired, looking for tag with the given tagID. Gets the TagID of the detected target.
     * If no targets with matching ID found, sets target variable to getBestTarget().
     * If no targets found, returns 0.
     * @param tagID The TagID to look for when updating the latest detected target.
     * @return The ID of the detected tag (or 0 if no targets detected).
     */
    public int getTagID(int tagID, boolean updateTarget){
        if(updateTarget) update(tagID);
        if(savedTarget == null) return 0;

        return savedTarget.getFiducialId();
    }

    /**
     * (NEEDS TESTING) Gets the Pose3D of the given AprilTag on the field using the 2024 AprilTagFieldLayout.
     * If tagID is invalid, returns null.
     * @param tagID
     * @return The Pose3D of the AprilTag with id tagID (or null if tagID invalid).
     */
    public Pose3d getTagPose3d(int tagID){
        Optional<Pose3d> result = aprilTagFieldLayout.getTagPose(tagID);
        if(result.isEmpty()) return null;
        return result.get();
    }

    /**
     * (NEEDS TESTING) Updates target if desired. Gets the Transform3D of the Camera in relation to the Target (AprilTag)
     * using target.getBestCameraToTarget().
     * If no target found, returns null.
     * @return The Transform3D of the Camera in relation to the latest Target (or null if target not found).
     */
    public Transform3d getBestCamToTarget(boolean updateTarget){
        if(!hasTargets(updateTarget)) return null;

        return savedTarget.getBestCameraToTarget();
    }

    /**
     * (NEEDS TESTING) Updates target if desired. Gets the Yaw on the field of the latest target (AprilTag) using target.getYaw().
     * If no target found, returns 0.
     * @return The yaw of the AprilTag(or 0 if no target found).
     */
    public double getYaw(boolean updateTarget){
        if(!hasTargets(updateTarget)) return 0;

        return savedTarget.getYaw();
    }

    /**
     * Updates target if desired. Gets the distance in meters from the robot to the latest target (AprilTag)
     * using PhotonUtils.calculateDistanceToTargetMeters().
     * If no target found, returns 0.
     * @return The distance (meters) from robot to latest target (or 0 if no target found).
     */
    public double getDistance(boolean updateTarget) {
        if(!hasTargets(updateTarget)) return 0;

        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch,
                Units.degreesToRadians(savedTarget.getPitch()));
        return distance;
    }

    /**
     * (NEEDS TESTING) Updates target if desired. Gets the Robot's Pose3D on the field using PhotonPoseEstimator.
     * If no target found, returns null.
     * If PhotonPoseEstimator fails to get the pose, returns a new Pose3D with empty coordinates (0, 0, 0) and rotation3d (0, 0, 0).
     * @return Estimated Pose3D of the robot on the field (or null if no target) (or empty Pose3D if calculation failed).
     */
    public Pose3d getPose3dFromPPE(boolean updateTarget){
        if(!hasTargets(updateTarget)) return null;
        
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        if(pose.isPresent()){
            return pose.get().estimatedPose;
        }
        else{
            return new Pose3d(0,0,0, new Rotation3d(0.0, 0.0, 0.0));
        }
        
    }

    /**
     * (NEEDS TESTING) Updates target if desired. Gets the Robot's Pose3D on the field using the Pose3D of the latest target (AprilTag).
     * The tag's Pose3D is offset by the camera's relative position to the tag (gotten from getBestCamToTarget()) and then offset by
     * the robot's relative position to the camera (robotToCam constant).
     * @return The Robot's Pose3D on the field.
     */
    public Pose3d getPose3dFromTarget(int tagID, boolean updateTarget){
        Pose3d tagPose3d = getTagPose3d(tagID);
        Transform3d camToTag = getBestCamToTarget(updateTarget);
        Pose3d robot = tagPose3d.plus(camToTag.inverse().plus(robotToCam.inverse()));
        return robot;
    }

    // /**
    //  * (NEEDS TESTING) Updates target if desired. Gets the Robot's Pose2D on the field using PhotonUtils.estimateFieldToRobot().
    //  * If no targets found, returns null.
    //  * @return The Robot's Pose2D on the field (or null if no targets).
    //  */
    // public Pose2d getPose2d(boolean updateTarget){
    //     if(!hasTargets(updateTarget)) return null;

    //     // This tries to use PhotonUtils.estimateFieldToRobot() to get the robot's Pose2d.
    //     // I followed the PhotonVision documentation:
    //     // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html

    //     // TO-DO: Test this on robot and fix anything that needs fixing for this to work.

    //     // these shouldn't need tuning.
    //     PhotonPipelineResult result = camera.getLatestResult();
    //     PhotonTrackedTarget target = null;
    //     if(result.hasTargets()) target = result.getBestTarget();
    //     if(target == null) return null;

    //     double targetPitch = Units.degreesToRadians(target.getPitch());
    //     Rotation2d targetYaw = Rotation2d.fromDegrees(-target.getYaw());

    //     // need the pose of the AprilTag. somehow use TargetID to get the pose.
    //     //int targetID = target.getFiducialId();
    //     Pose2d targetPose = new Pose2d(-2, 3, new Rotation2d());

    //     // need to tune this constant later
    //     Transform2d cameraToRobot = new Transform2d(0.5, 0.5, new Rotation2d());

    //     // PhotonVision's built in Pose2D estimator. Need to have actual parameters.
    //     // the gyro.getRotation2d() has to be rechecked; not sure if this is the right way to implement.
    //     Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
    //         cameraHeight, targetHeight, cameraPitch, targetPitch, targetYaw, gyro.getRotation2d(), targetPose, cameraToRobot
    //     );

    //     // for testing: try displaying the robotPose
    //     return robotPose;
    // }
    
    public void periodic(){
        //System.out.println("Testing the output in terminal");
        try{
            SmartDashboard.putNumber("Tag ID", getTagID(true));
            // SmartDashboard.putNumber("PoseX", getRobotPose3d().getX());
            // SmartDashboard.putNumber("PoseY", getRobotPose3d().getY());
            // SmartDashboard.putNumber("PoseZ", getRobotPose3d().getZ());
            // SmartDashboard.putNumber("Rot Z", getRobotPose3d().getRotation().getAngle());
            
        }catch(Exception e) {

            // StringWriter sw = new StringWriter();
            // PrintWriter pw = new PrintWriter(sw);
            // e.printStackTrace(pw);
            // String sStackTrace = sw.toString(); // stack trace as a string
            // SmartDashboard.putString("Exception yar", sStackTrace);
            // SmartDashboard.putNumber("PoseX", 0);
            // SmartDashboard.putNumber("PoseY", 0);
            // SmartDashboard.putNumber("PoseZ", 0);
            // SmartDashboard.putNumber("Rot Z", 0);
        }
    }

}
