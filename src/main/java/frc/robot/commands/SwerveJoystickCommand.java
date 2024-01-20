package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveRequestFieldCentric;

public class SwerveJoystickCommand extends Command {
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, speedIncreaseControlFunction,
            speedDecreaseControlFunction;

    // Limits rate of change (in this case x, y, and turning movement)
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final SwerveSubsystem swerveDrivetrain;

    // Sets everything
    public SwerveJoystickCommand(
            Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction, Supplier<Double> turningSpdFunction,
            Supplier<Double> speedIncreaseControlFunction, Supplier<Double> speedDecreaseControlFunction,
            SwerveSubsystem csd) {

        this.xSpdFunction = frontBackFunction;
        this.ySpdFunction = leftRightFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.speedIncreaseControlFunction = speedIncreaseControlFunction;
        this.speedDecreaseControlFunction = speedDecreaseControlFunction;
        this.xLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.swerveDrivetrain = csd;
        
        // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
        addRequirements(swerveDrivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get(); // xSpeed is actually front back (front +, back -)
        double ySpeed = ySpdFunction.get(); // ySpeed is actually left right (left +, right -)
        double turningSpeed = turningSpdFunction.get(); // turning speed is (anti-clockwise +, clockwise -)

        // 2. Normalize inputs
        double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
        if (length > 1d) {
            length = Math.sqrt(length);
            xSpeed /= length;
            ySpeed /= length;
        }

        // 3. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.OI.kLeftJoystickDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OI.kLeftJoystickDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OI.kRightJoystickDeadband ? turningSpeed : 0.0;

        // 4. Make the driving smoother
        // Speed control
        double driveSpeed = (Constants.Swerve.kTeleDriveMaxPercentSpeed - Constants.Swerve.kTeleDriveMinPercentSpeed)
                * (speedIncreaseControlFunction.get() - speedDecreaseControlFunction.get())
                + Constants.Swerve.kTeleDriveMinPercentSpeed;

        // Applies slew rate limieter
        xSpeed = xLimiter.calculate(xSpeed) * driveSpeed * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * driveSpeed * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * driveSpeed
                * Constants.Swerve.kPhysicalMaxAngularSpeedRadiansPerSecond;

        // Final values to apply to drivetrain
        final double x = xSpeed;
        final double y = ySpeed;
        final double turn = turningSpeed;

        // 5. Applying the drive request on the swerve drivetrain
        // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
        final SwerveRequest.FieldCentric drive = new SwerveRequestFieldCentric()
                // .withDeadband(Constants.kPhysicalMaxSpeedMetersPerSecond *
                // 0.01).withRotationalDeadband(Constants.kPhysicalMaxAngularSpeedRadiansPerSecond
                // * 0.01) // old deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // OPEN LOOP CONTROL

        // Applies request
        this.swerveDrivetrain.setControl(drive
                .withVelocityX(x)
                .withVelocityY(y) 
                .withRotationalRate(turn)
                // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                // .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                );
    } // Drive counterclockwise with negative X (left))

    @Override
    public void end(boolean interrupted) {
        // Applies SwerveDriveBrake (brakes the robot by turning wheels)
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        this.swerveDrivetrain.applyRequest(() -> brake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}