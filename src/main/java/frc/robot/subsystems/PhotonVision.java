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
   private static PhotonVision instance; 

   private PhotonCamera camera = new PhotonCamera("FrontCam");
   
   private AprilTagFieldLayout aprilTagFieldLayout;
   private PhotonPoseEstimator photonPoseEstimator;
   private PhotonPipelineResult pResult;
   private PhotonTrackedTarget target;

   private static double cameraHeight = 0.3;
   private static double targetHeight = 1;
   private static double cameraPitch = Math.PI/8;
   private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, -0.25, cameraHeight), new Rotation3d(0, cameraPitch, 0));
 
   // Need to set up gyro correctly; this is most likely wrong
   AnalogGyro gyro = new AnalogGyro(0);

//    private DoubleLogEntry pitchLog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/pitch");
//    private DoubleLogEntry yawlog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/yaw");
//    private DoubleLogEntry distancelog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/distance");
//    private DoubleLogEntry poselog = new DoubleLogEntry(DataLogManager.getLog(),  "/log/input/pose");

    /**
     * PhotonVision constructor (private).
     * @apiNote Initializes AprilTagFieldLayout
     * @apiNote Initializes PhotonPoseEstimator
     * @apiNote Runs camera.getLatestResult()
     * @apiNote Updates target (pResult.getBestTarget())
     */
   private PhotonVision(){
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
    pResult = camera.getLatestResult();
    target = pResult.getBestTarget();
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
     * Updates the camera pipeline and the best target variable. Target not guaranteed to exist.
     */
    public void update(){
        pResult = camera.getLatestResult();
        target = pResult.getBestTarget();
    }

    /**
     * Checks to see if the target from the latest result exists.
     * @return Whether or not a target exists
     */
    public boolean foundTarget(){
        boolean hasTargets = pResult.hasTargets();
        //if(!hasTargets) System.out.println("ERROR: No Targets Found");
        return hasTargets;
    }

    /**
     * (NEEDS TESTING) Checks to see if the latest detected target matches IDs with the given tag ID.
     * @param tagID - The tagID to compare with
     * @return Whether or not the latest target ID matches the given tagID.
     */
    public boolean matchesTagID(int tagID){
        int current = getTagID();
        return current == tagID;
    }

    /**
     * (NEEDS TESTING) Updates target. Gets the TagID of the latest detected target. Returns 0 if no targets found.
     * @return The tagID of the latest detected target (or 0 if no targets found).
     */
    public int getTagID(){
        update();
        if(!foundTarget()) return 0;

        return target.getFiducialId();
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
     * (NEEDS TESTING) Updates target. Gets the Transform3D of the Camera in relation to the Target (AprilTag)
     * using target.getBestCameraToTarget().
     * If no target found, returns null.
     * @return The Transform3D of the Camera in relation to the latest Target (or null if target not found).
     */
    public Transform3d getBestCamToTarget(){
        update();
        if(!foundTarget()) return null;

        return target.getBestCameraToTarget();
    }

    /**
     * (NEEDS TESTING) Updates target. Gets the Yaw on the field of the latest target (AprilTag) using target.getYaw().
     * If no target found, returns 0.
     * @return The yaw of the AprilTag(or 0 if no target found).
     */
    public double getYaw(){
        update();
        if(!foundTarget()) return 0;

        return target.getYaw();
    }

    /**
     * Updates target. Gets the distance in meters from the robot to the latest target (AprilTag)
     * using PhotonUtils.calculateDistanceToTargetMeters().
     * If no target found, returns 0.
     * @return The distance (meters) from robot to latest target (or 0 if no target found).
     */
    public double getDistance() {
        update();
        if(!foundTarget()) return 0;

        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch,
                Units.degreesToRadians(target.getPitch()));
        return distance;
    }

    /**
     * (NEEDS TESTING) Updates target. Gets the Robot's Pose3D on the field using PhotonPoseEstimator. If no target found, returns null.
     * If PhotonPoseEstimator fails to get the pose, returns a new Pose3D with empty coordinates (0, 0, 0) and rotation3d (0, 0, 0).
     * @return Estimated Pose3D of the robot on the field (or null if no target) (or empty Pose3D if calculation failed).
     */
    public Pose3d getRobotPose3d(){
        update();
        if(!foundTarget()) return null;
        
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        if(pose.isPresent()){
            return pose.get().estimatedPose;
        }
        else{
            return new Pose3d(0,0,0, new Rotation3d(0.0, 0.0, 0.0));
        }
        
    }

    /**
     * (NEEDS TESTING) Updates target. Gets the Robot's Pose2D on the field using PhotonUtils.estimateFieldToRobot().
     * If no targets found, returns null.
     * @return The Robot's Pose2D on the field (or null if no targets).
     */
    public Pose2d getRobotPose2d(){
        update();
        if(!foundTarget()) return null;

        // This tries to use PhotonUtils.estimateFieldToRobot() to get the robot's Pose2d.
        // I followed the PhotonVision documentation:
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html

        // TO-DO: Test this on robot and fix anything that needs fixing for this to work.

        // these shouldn't need tuning.
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = null;
        if(result.hasTargets()) target = result.getBestTarget();
        if(target == null) return null;

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
        //System.out.println("Testing the output in terminal");
        try{
            SmartDashboard.putNumber("Tag ID", getTagID());
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
