package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RitvikSetty {
    private Pose2d robotPose;
    private Rotation2d armAngle;

    /**
     * 
     * @param robotPose Robot Pose
     * @param armAngle Angle of the Arm
     */
    public RitvikSetty (Pose2d robotPose, Rotation2d armAngle) {
        this.robotPose = robotPose;
        this.armAngle = armAngle; 
    }

    /**
     * 
     * @param updatedPose Updated pose value
     */
    public void setRobotPose(Pose2d updatedPose) {
        this.robotPose = updatedPose;
    }
   
    /**
     * 
     * @return Get the pose value
     */
    public Pose2d getPose() {
        return this.robotPose;
    }

    /**
     * 
     * @param updatedRotation Updated arm rotation angle
     */
    public void setArmAngle(Rotation2d updatedRotation){
        this.armAngle=updatedRotation;
    }

    /**
     * 
     * @return Get the rotation angle value
     */
    public Rotation2d getArmAngle() {
        return this.armAngle;
    }
}