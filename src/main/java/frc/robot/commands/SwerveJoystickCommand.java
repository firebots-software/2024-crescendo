package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveRequestFieldCentricSwerveOptimized;
import java.util.function.Supplier;

public class SwerveJoystickCommand extends Command {
  private final Supplier<Double> xSpdFunction,
      ySpdFunction,
      turningSpdFunction,
      speedIncreaseControlFunction,
      speedDecreaseControlFunction;

  // Limits rate of change (in this case x, y, and turning movement)
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private final SwerveSubsystem swerveDrivetrain;

  // Sets everything
  public SwerveJoystickCommand(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Double> speedIncreaseControlFunction,
      Supplier<Double> speedDecreaseControlFunction,
      SwerveSubsystem swerveSubsystem) {

    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.speedIncreaseControlFunction = speedIncreaseControlFunction;
    this.speedDecreaseControlFunction = speedDecreaseControlFunction;
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

    // 3. Apply deadband
    xSpeed = Math.abs(xSpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? ySpeed : 0.0;
    turningSpeed =
        Math.abs(turningSpeed) > Constants.OI.RIGHT_JOYSTICK_DEADBAND ? turningSpeed : 0.0;

    // 4. Make the driving smoother
    // There are two triggers to change the speed of the swerve driverbase.
    // We use both inputs to find how fast the swerve drivebase should be going.
    double driveSpeed =
        (Constants.Swerve.TELE_DRIVE_MAX_PERCENT_SPEED
                    - Constants.Swerve.TELE_DRIVE_MIN_PERCENT_SPEED)
                * (speedIncreaseControlFunction.get() - speedDecreaseControlFunction.get())
            + Constants.Swerve.TELE_DRIVE_MIN_PERCENT_SPEED;

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
        new SwerveRequestFieldCentricSwerveOptimized()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // OPEN LOOP CONTROL

    // Applies request
    this.swerveDrivetrain.setControl(
        drive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn));
  } // Drive counterclockwise with negative X (left))

  @Override
  public void end(boolean interrupted) {
    // Applies SwerveDriveBrake (brakes the robot by turning wheels)
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    this.swerveDrivetrain.getSwerveRequest(() -> brake);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
