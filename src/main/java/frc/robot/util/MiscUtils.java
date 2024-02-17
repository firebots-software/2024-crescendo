package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class MiscUtils {
  /**
   * @param point The Pose2d that needs to be reflected across the midline
   * @return The reflected Pose2d (Midline is at x = 8.27 field-based)
   */
  public static Pose2d reflectAcrossMidline(Pose2d point) {
    return new Pose2d(
        point.getX() - (2.0 * (point.getX() - Constants.Landmarks.CENTER_LINE_LOCATION)),
        point.getY(),
        new Rotation2d(Math.PI - point.getRotation().getRadians()));
  }

  /**
   * 
   * Returns the angle between Robot's current angle and angle to face target
   * 
   * @param robotPose The Pose2d of the robot
   * @param targetPose The Pose2d of the taret location
   * @return
   */
  public static Double getAlignmentRotation(Pose2d robotPose, Pose2d targetPose){
    double robot_x = robotPose.getX();
    double robot_y = robotPose.getY();
    double robot_rotation =
        ((robotPose.getRotation().getRadians()) + Math.PI * 2 ) % Math.PI * 2;

    double target_x = targetPose.getX();
    double target_y = targetPose.getY();

    double angle;
    if (target_y != robot_x) {
      angle = (Math.atan2((target_y - robot_y), (target_x - robot_x)) + Math.PI * 2) % Math.PI * 2;
    } else {
      angle = robot_rotation; // do not turn
    }
    return angle;
  }
}
