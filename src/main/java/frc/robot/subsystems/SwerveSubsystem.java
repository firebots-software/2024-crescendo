package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    
    private static SwerveSubsystem instance;

    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRotations
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRotations
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRotations
    );

     private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRotations
    );
   
    private final Pigeon2 gyro = new Pigeon2(DriveConstants.PIGEON_PORT);
    private double flatPitch = 0;
    
    private final edu.wpi.first.math.kinematics.SwerveDriveOdometry odometer = new edu.wpi.first.math.kinematics.SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new edu.wpi.first.math.geometry.Rotation2d(0), 
        getModulePositions()
    );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public void setHeading(double deg) {
        gyro.setYaw(deg);
    }

    public void setPitch(double deg) {
        flatPitch = deg;
    }

    public void zeroPitch() {
        flatPitch = getPitch();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360); // TODO: maybe should be negative?
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble() - flatPitch;
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public edu.wpi.first.math.geometry.Rotation2d getRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(), backRight.getModulePosition()};
    }

    public edu.wpi.first.math.geometry.Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Pitch", getPitch());
        SmartDashboard.putNumber("Robot Roll", getRoll());
        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("front left encoder", frontLeft.getDrivingTickValues());
        SmartDashboard.putNumber("front right encoder", frontRight.getDrivingTickValues());
        SmartDashboard.putNumber("back left encoder", backLeft.getDrivingTickValues());
        SmartDashboard.putNumber("back right encoder", backRight.getDrivingTickValues());
    } 

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // System.out.println(desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,  DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setTurning(double rad) {
        frontLeft.setTurnState(rad);
        frontRight.setTurnState(rad);
        backLeft.setTurnState(rad);
        backRight.setTurnState(rad);
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    public void resetEncoders(){
        frontLeft.setDrivingTickValues(0);
        frontRight.setDrivingTickValues(0);
        backLeft.setDrivingTickValues(0);
        backRight.setDrivingTickValues(0);
    }

    

}
