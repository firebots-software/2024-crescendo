package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldCentricOptimizedSwerve extends FieldCentric {
  // Apply function that does module optimization
  @Override
  public StatusCode apply(
      SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    // Elements of the request to be applied
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    double toApplyOmega = RotationalRate;

    // Apply deadbands for SwerveDrivetrain movements
    if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
      toApplyX = 0;
      toApplyY = 0;
    }

    if (Math.abs(toApplyOmega) < RotationalDeadband) {
      toApplyOmega = 0;
    }

    // Constructs Chassis Speeds for given request
    ChassisSpeeds speeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
            parameters.updatePeriod);
            
    // Chassis Speeds conversion to individual module states
    var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());
    
    // Applies module states individually + uses optimization
    SmartDashboard.putNumber("FrontLeftModuleTurnAngleToSet", states[0].angle.getDegrees());
    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(
          SwerveModuleState.optimize(states[i], states[i].angle),
          DriveRequestType,
          SteerRequestType);
    }

    return StatusCode.OK;
  }
}
