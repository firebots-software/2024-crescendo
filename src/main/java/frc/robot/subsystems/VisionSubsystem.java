package frc.robot.subsystems;

import java.util.Comparator;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Wrapper for PhotonCamera class */
public class VisionSubsystem extends PhotonCamera implements Subsystem {

    private static final String DEFAULT_CAM_NAME = "ObjDetectionCam";
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(18.0);
    private final double CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(-25.0).getRadians();
    private final double TARGET_HEIGHT_METERS = 0.0; 
    private final NetworkTable noteTable;
    private final NetworkTableEntry noteAngleEntry;
    private final NetworkTableEntry noteFoundEntry;
    private final NetworkTableEntry noteDistanceEntry;
    private double noteAngle = 0.0;
    private static VisionSubsystem vs;

    private VisionSubsystem() {
        super(DEFAULT_CAM_NAME);
        noteTable = NetworkTableInstance.getDefault().getTable("Note");
        noteAngleEntry = noteTable.getEntry("targetAngle");
        noteFoundEntry = noteTable.getEntry("noteFound");
        noteDistanceEntry = noteTable.getEntry("targetDistance");
    }

    public static VisionSubsystem getInstance() {
        if(vs == null){
            vs = new VisionSubsystem();
        }
        return vs;
    }

    public void periodic() {
        PhotonPipelineResult result = getLatestResult();
        
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            
            // Runs through the list of targets and gets the one with the highest y coord and closest to center x (bottom center of camera)
            PhotonTrackedTarget bestTarget = targets.stream()
            .max(Comparator.<PhotonTrackedTarget>comparingDouble(target -> {
                List<TargetCorner> cornersList = target.getDetectedCorners();
            
                double avgX = 0;
                double avgY = 0;
                for (TargetCorner i : cornersList) {
                    avgX += i.x;
                    avgY += i.y;
                }
                
                // Calculate average X and Y
                avgX = (avgX) / 4.0;
                avgY = (avgY) / 4.0;
                
                double xCenterOffset = Math.abs(avgX - 0.5);
                return avgY - (xCenterOffset * 2); // Higher Y values (lower in frame) are preferred, subtracts the X offset from center
            }))
            .orElse(null);
            
            if (bestTarget != null) {
                // Calculate yaw (horizontal angle) to target
                double yaw = bestTarget.getYaw();
                noteAngle = bestTarget.getYaw();
                double distance = getDistanceToTarget();

                noteAngleEntry.setDouble(yaw);
                noteFoundEntry.setBoolean(true);
                noteDistanceEntry.setDouble(distance);
            }
        } else {
            noteFoundEntry.setBoolean(false);
            noteAngleEntry.setDouble(0.0);
            noteDistanceEntry.setDouble(0.0);
        }

    }

    public double getYaw() {
        return noteAngle;
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            return range;
        }
        return 0.0;
    }
    
    public Transform3d transformToNote() {
        Transform3d pose = getLatestResult().getBestTarget().getBestCameraToTarget();
        return pose;
        // robot_pose.transformBy(robot_to_camera).transformBy(transformToNote())
    }
}