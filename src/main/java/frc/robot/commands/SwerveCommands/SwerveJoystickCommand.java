package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveJoystickCommand extends Command {
  protected final Supplier<Double> xSpdFunction,
      ySpdFunction,
      turningSpdFunction,
      speedControlFunction;

  protected final Supplier<Boolean> fieldRelativeFunction;

  // Limits rate of change (in this case x, y, and turning movement)
  protected final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  protected final SwerveSubsystem swerveDrivetrain;

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public SwerveJoystickCommand(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Double> speedControlFunction,
      Supplier<Boolean> fieldRelativeFunction,
      SwerveSubsystem swerveSubsystem) {
    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.speedControlFunction = speedControlFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;

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

  // Sets everything, not field relative
  public SwerveJoystickCommand(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Double> speedControlFunction,
      SwerveSubsystem swerveSubsystem) {

    this(
        frontBackFunction,
        leftRightFunction,
        turningSpdFunction,
        speedControlFunction,
        () -> false,
        swerveSubsystem);
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

    // 2. Normalize inputs
    double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
    if (length > 1d) {
      length = Math.sqrt(length);
      xSpeed /= length;
      ySpeed /= length;
    }

    //Apply Square (will be [0,1] since `speed` is [0,1])
    xSpeed = Math.pow(xSpeed,2) * xSpeed < 0 ? -1 : 1;
    ySpeed = Math.pow(ySpeed,2) * ySpeed < 0 ? -1 : 1;


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
    SwerveRequest drive =
        !fieldRelativeFunction.get()
            ? fieldCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn)
            : robotCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn);

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
