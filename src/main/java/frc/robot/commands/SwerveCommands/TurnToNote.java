package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnToNote extends Command {
    protected final Supplier<Double> xSpdFunction,
    ySpdFunction,
    turningSpdFunction,
    speedControlFunction;

    protected final Supplier<Boolean> fieldRelativeFunction;

    // Limits rate of change (in this case x, y, and turning movement)
    protected final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final SwerveSubsystem swerve;
    private final VisionSubsystem camera;
    private Supplier<Double> frontBackFunction;
    private Supplier<Double> leftRightFunction;
    private static final PIDController turningPID = new PIDController(1d, 0.002, 0.01d);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable table = inst.getTable("photonvision/ObjDetectionCam");
    private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    private boolean squaredTurn;
    protected final SwerveSubsystem swerveDrivetrain;

    public TurnToNote(SwerveSubsystem swerve, VisionSubsystem camera, Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Double> speedControlFunction,
      Supplier<Boolean> fieldRelativeFunction,
      SwerveSubsystem swerveSubsystem) {
        this.swerve = swerve;
        this.camera = camera;
        this.frontBackFunction = frontBackFunction;
        this.leftRightFunction = leftRightFunction;
        this.xSpdFunction = frontBackFunction;
        this.ySpdFunction = leftRightFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.speedControlFunction = speedControlFunction;
        this.fieldRelativeFunction = fieldRelativeFunction;
        this.squaredTurn = true;
        this.xLimiter =
            new SlewRateLimiter(Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter =
            new SlewRateLimiter(Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter =
            new SlewRateLimiter(Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        this.swerveDrivetrain = swerveSubsystem;
        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        double yawAngle = table.getEntry("targetYaw").getDouble(0);
        boolean noteFound = table.getEntry("hasTarget").getBoolean(false); //TODO: check the actual entry name, otherwise wont work
        
        if (noteFound) {
            DogLog.log("noteFound", noteFound);
            DogLog.log("Yawangle", yawAngle);
        
            // Rotation2d currentRotation = swerve.getState().Pose.getRotation();
            // Rotation2d targetRotation = currentRotation.plus(
            //     Rotation2d.fromDegrees(yawAngle)
            // );
            
            // Rotation2d actualTarget = turnTarget.get().rotateBy(Rotation2d.fromDegrees(90));
            //   Rotation2d computedError = actualTarget.minus(getSwerveRotation(swerveSubsystem));

          double computedRotation = turningPID.calculate(yawAngle / 180 * Math.PI);//TODO: Convert yawAngle to Radians
          computedRotation = MathUtil.clamp(computedRotation, -0.4, 0.4);
          if (Math.abs(yawAngle) < 1) {
            computedRotation = 0;
          }

        //   return -computedRotation;
            
            // 1. Get real-time joystick inputs
            double xSpeed = xSpdFunction.get(); // xSpeed is actually front back (front +, back -)
            double ySpeed = ySpdFunction.get(); // ySpeed is actually left right (left +, right -)
            double turningSpeed =
                computedRotation; // turning speed is (anti-clockwise +, clockwise -)

            // 2. Normalize inputs
            double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
            if (length > 1d) {
            length = Math.sqrt(length);
            xSpeed /= length;
            ySpeed /= length;
            }

            // Apply Square (will be [0,1] since `speed` is [0,1])
            xSpeed = xSpeed * xSpeed * Math.signum(xSpeed);
            ySpeed = ySpeed * ySpeed * Math.signum(ySpeed);
            if (squaredTurn) {
            turningSpeed = turningSpeed * turningSpeed * Math.signum(turningSpeed);
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

            // Applies slew rate limiter
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
            final double x = -xSpeed;
            final double y = -ySpeed;
            final double turn = turningSpeed;
            DogLog.log("TurnSpeed", turn);
            // 5. Applying the drive request on the swerve drivetrain
            // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
            SwerveRequest drive = robotCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn);

            // =
            //     !fieldRelativeFunction.get()
            //         ? fieldCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn)
            //         : 

            // Applies request
            this.swerveDrivetrain.setControl(drive);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}