package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends Command{

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, speedControlFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final SwerveSubsystem swerveDrivetrain;

    public SwerveJoystickCommand(
        Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction, Supplier<Double> turningSpdFunction,
        Supplier<Double> speedControlFunction, SwerveSubsystem csd) {
        
        this.xSpdFunction = frontBackFunction;
        this.ySpdFunction = leftRightFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.speedControlFunction = speedControlFunction;
        this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.swerveDrivetrain=csd;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get(); // xSpeed is actually front back (front +, back -)
        double ySpeed = ySpdFunction.get(); // ySpeed is actually left right (left +, right -)
        double turningSpeed = turningSpdFunction.get(); // turning speed is anti-clockwise +, clockwise -

        // 2. Normalize inputs
        double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
        if (length > 1d) { 
            length = Math.sqrt(length);
            xSpeed /= length;
            ySpeed /= length;
        }

        // 3. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.001 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.001 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.001 ? turningSpeed : 0.0;

        // 4. Make the driving smoother
        double driveSpeed = (Constants.kTeleDriveMaxPercentSpeed - Constants.kTeleDriveMinPercentSpeed)
        * speedControlFunction.get() + Constants.kTeleDriveMinPercentSpeed;

        xSpeed = xLimiter.calculate(xSpeed) * driveSpeed * Constants.kPhysicalMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * driveSpeed * Constants.kPhysicalMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * driveSpeed * Constants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        final double x = xSpeed;
        final double y = ySpeed;
        final double turn = turningSpeed;


        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.kPhysicalMaxSpeedMetersPerSecond * 0.1).withRotationalDeadband(Constants.kPhysicalMaxAngularSpeedRadiansPerSecond * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

        this.swerveDrivetrain.applyRequest(() -> drive.withVelocityX(-x)
            .withVelocityY(-y) // Drive left with negative X (left)
            .withRotationalRate(-turn)); // Drive counterclockwise with negative X (left))

    }

    @Override
    public void end(boolean interrupted) {
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        this.swerveDrivetrain.applyRequest(() -> brake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}