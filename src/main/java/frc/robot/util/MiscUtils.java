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
}
