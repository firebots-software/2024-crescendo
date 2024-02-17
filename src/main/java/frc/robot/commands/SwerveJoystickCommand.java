package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldCentricOptimizedSwerve;
import java.util.function.Supplier;

public class SwerveJoystickCommand extends Command {
  private final Supplier<Double> xSpdFunction,
      ySpdFunction,
      turningSpdFunction,
      speedControlFunction;
  
      private final Supplier<Boolean> automaticTurnFunction;

  // Limits rate of change (in this case x, y, and turning movement)
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private final SwerveSubsystem swerveDrivetrain;

  // Sets everything
  public SwerveJoystickCommand(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Double> speedControlFunction,
      Supplier<Boolean> automaticTurnFunction,
      SwerveSubsystem swerveSubsystem) {
    
    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.speedControlFunction = speedControlFunction;
    this.automaticTurnFunction = automaticTurnFunction;
    this.xLimiter =
        new SlewRateLimiter(Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    this.yLimiter =
        new SlewRateLimiter(Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    this.turningLimiter =
        new SlewRateLimiter(Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
    this.swerveDrivetrain = swerveSubsystem;

    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

    public Double rotateToSpeaker() {
    SmartDashboard.putNumber("turning", turningSpdFunction.get());
    // if (doSpin.get() > 0) {
    double robot_x = this.swerveDrivetrain.getState().Pose.getX();
    double robot_y = this.swerveDrivetrain.getState().Pose.getY();
    double robot_rotation =
        (this.swerveDrivetrain.getState().Pose.getRotation().getRadians()) % Math.PI * 2;

    double speaker_x = Constants.Landmarks.SPEAKER_LOCATION.getX();
    double speaker_y = Constants.Landmarks.SPEAKER_LOCATION.getY();

    SmartDashboard.putNumber("robot_x", robot_x);
    SmartDashboard.putNumber("robot_y", robot_y);
    // SmartDashboard.putNumber("robot_rotation", robot_rotation);
    SmartDashboard.putNumber("speaker_x", speaker_x);
    SmartDashboard.putNumber("speaker_y", speaker_y);

    double angle;
    if (speaker_x != robot_x) {
      angle = (Math.atan2((speaker_y - robot_y), (speaker_x - robot_x))) % Math.PI * 2;
    } else {
      angle = robot_rotation; // do not turn
    }

    SmartDashboard.putNumber("angle", angle);

    return Constants.Swerve.AUTONOMOUS_TURNING.calculate(robot_rotation,angle);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get(); // xSpeed is actually front back (front +, back -)
    double ySpeed = ySpdFunction.get(); // ySpeed is actually left right (left +, right -)
    double turningSpeed =
        turningSpdFunction.get(); // turning speed is (anti-clockwise +, clockwise -)
    if(automaticTurnFunction.get()){
      turningSpeed = rotateToSpeaker();
    }
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
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(x)
            .withVelocityY(y)
            .withRotationalRate(turn);
            

    // Applies request
    this.swerveDrivetrain.setControl(drive);
  } // Drive counterclockwise with negative X (left))

  @Override
  public void end(boolean interrupted) {
    // Applies SwerveDriveBrake (brakes the robot by turning wheels)
    this.swerveDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
