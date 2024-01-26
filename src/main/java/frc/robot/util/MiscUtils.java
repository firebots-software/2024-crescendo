package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class MiscUtils {
    public static Pose2d reflectAcrossMidline(Pose2d point) {
        return new Pose2d(point.getX()+(2.0*(point.getX()-Constants.Landmarks.centerLine)), point.getY(), new Rotation2d(point.getRotation().getDegrees()+(2.0*(90.0-point.getRotation().getDegrees()))));
    }
}
