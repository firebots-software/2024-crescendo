package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldCentricOptimizedSwerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class TurnToSpeaker extends SwerveJoystickCommand {
  public Supplier<Double> doSpin;
  public static Pose2d speakerPos = new Pose2d(0, 5.5, new Rotation2d());
  PIDController pid = new PIDController(0.35, 0, 0); // placeholder vals

  public TurnToSpeaker(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Double> speedControlFunction,
      SwerveSubsystem swerveSubsystem) {
    super(
        frontBackFunction,
        leftRightFunction,
        turningSpdFunction,
        speedControlFunction,
        swerveSubsystem);

    this.doSpin = turningSpdFunction;
  }

  public Double rotation() {
    SmartDashboard.putNumber("turnning", doSpin.get());
    if (doSpin.get() > 0) {
      double robot_x = this.swerveDrivetrain.getState().Pose.getX();
      double robot_y = this.swerveDrivetrain.getState().Pose.getY();
      double robot_rotation = this.swerveDrivetrain.getState().Pose.getRotation().getRadians() % Math.PI*2;

      double speaker_x = speakerPos.getX();
      double speaker_y = speakerPos.getY();

      SmartDashboard.putNumber("robot_x", robot_x);
      SmartDashboard.putNumber("robot_y", robot_y);
      SmartDashboard.putNumber("robot_rotation", robot_rotation);
      SmartDashboard.putNumber("speaker_x", speaker_x);
      SmartDashboard.putNumber("speaker_y", speaker_y);

      double angle;
      if (speaker_x != robot_x) {
        angle = Math.atan(Math.abs((speaker_y - robot_y) / (speaker_x - robot_x)));
      } else {
        angle = 0;
      }

      SmartDashboard.putNumber("angle", angle);

      return pid.calculate(robot_rotation, angle);
    } else {
      return 0d;
    }
  }

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get(); // xSpeed is actually front back (front +, back -)
    double ySpeed = ySpdFunction.get(); // ySpeed is actually left right (left +, right -)
    double turningSpeed = rotation();

    // 2. Normalize inputs
    double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
    if (length > 1d) {
      length = Math.sqrt(length);
      xSpeed /= length;
      ySpeed /= length;
    }

    // 3. Apply deadband
    xSpeed = Math.abs(xSpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? ySpeed : 0.0;
    turningSpeed =
        Math.abs(turningSpeed) > Constants.OI.RIGHT_JOYSTICK_DEADBAND ? turningSpeed : 0.0;

    // 4. Make the driving smoother
    // This is a double between TELE_DRIVE_SLOW_MODE_SPEED_PERCENT and
    // TELE_DRIVE_FAST_MODE_SPEED_PERCENT
    double driveSpeed =
        (Constants.Swerve.TELE_DRIVE_PERCENT_SPEED_RANGE * (speedControlFunction.get()))
            + Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT;

    // Applies slew rate limieter
    xSpeed =
        xLimiter.calculate(xSpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed =
        yLimiter.calculate(ySpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    turningSpeed =
        turningLimiter.calculate(turningSpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;

    // Final values to apply to drivetrain
    final double x = xSpeed;
    final double y = ySpeed;
    final double turn = turningSpeed;

    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    final SwerveRequest.FieldCentric drive =
        new FieldCentricOptimizedSwerve()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(x)
            .withVelocityY(y)
            .withRotationalRate(turn); // OPEN LOOP CONTROL

    // Applies request
    this.swerveDrivetrain.setControl(drive);
  } // Drive counterclockwise with negative X (left))



  // public Double speed(PIDController pidController) {

  // }

}
