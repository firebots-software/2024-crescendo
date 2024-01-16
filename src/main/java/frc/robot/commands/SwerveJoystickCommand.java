package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends Command{

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction,speedIncreaseControlFunction, speedDecreaseControlFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final SwerveSubsystem swerveDrivetrain;

    public SwerveJoystickCommand(
        Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction, Supplier<Double> turningSpdFunction,
        Supplier<Double> speedIncreaseControlFunction, Supplier<Double> speedDecreaseControlFunction, SwerveSubsystem csd) {
        
        this.xSpdFunction = frontBackFunction;
        this.ySpdFunction = leftRightFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.speedIncreaseControlFunction = speedIncreaseControlFunction;
        this.speedDecreaseControlFunction = speedDecreaseControlFunction;
        this.xLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.swerveDrivetrain=csd;

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
        double turningSpeed = turningSpdFunction.get(); // turning speed is anti-clockwise +, clockwise -

        // 2. Normalize inputs
        double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
        if (length > 1d) { 
            length = Math.sqrt(length);
            xSpeed /= length;
            ySpeed /= length;
        }

        // 3. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.01 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.01 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.01 ? turningSpeed : 0.0;

        // 4. Make the driving smoother
        double driveSpeed = (Constants.Swerve.kTeleDriveMaxPercentSpeed - Constants.Swerve.kTeleDriveMinPercentSpeed)
        * (speedIncreaseControlFunction.get() - speedDecreaseControlFunction.get()) + Constants.Swerve.kTeleDriveMinPercentSpeed;

        xSpeed = xLimiter.calculate(xSpeed) * driveSpeed * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * driveSpeed * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * driveSpeed * Constants.Swerve.kPhysicalMaxAngularSpeedRadiansPerSecond;
        final double x = xSpeed;
        final double y = ySpeed;
        final double turn = turningSpeed;


        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        // .withDeadband(Constants.kPhysicalMaxSpeedMetersPerSecond * 0.01).withRotationalDeadband(Constants.kPhysicalMaxAngularSpeedRadiansPerSecond * 0.01) // old deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

        SmartDashboard.putNumber("Drive X Velocity",drive.VelocityX);
        SmartDashboard.putNumber("Drive Y Velocity",drive.VelocityY);
        SmartDashboard.putNumber("Rotational Speed",drive.RotationalRate);

        this.swerveDrivetrain.setControl(drive
            .withVelocityX(x)
            .withVelocityY(y) // Drive left with negative X (left)
            .withRotationalRate(turn)
        );} // Drive counterclockwise with negative X (left))

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