package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OI;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, speedControlFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
        Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction, Supplier<Double> turningSpdFunction,
        Supplier<Double> speedControlFunction, Supplier<Boolean> fieldOrientedFunction) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = frontBackFunction;
        this.ySpdFunction = leftRightFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.speedControlFunction = speedControlFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
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
        xSpeed = Math.abs(xSpeed) > OI.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OI.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OI.DEADBAND ? turningSpeed : 0.0;

        // 4. Make the driving smoother
        double driveSpeed = (DriveConstants.kTeleDriveMaxPercentSpeed - DriveConstants.kTeleDriveMinPercentSpeed)
        * speedControlFunction.get() + DriveConstants.kTeleDriveMinPercentSpeed;

        xSpeed = xLimiter.calculate(xSpeed) * driveSpeed * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * driveSpeed * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * driveSpeed * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

        // 5. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 6. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 7. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}