package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModuleSubsystem extends SubsystemBase {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final CANcoder turningEncoder;

    private final PIDController turningPidController;

    private final double absoluteEncoderOffsetRad;
    private final double absoluteDriveEncoderOffset;

    public SwerveModuleSubsystem(int driveMotorId, int turningMotorId, int CANCoderId, double absoluteEncoderOffset) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);
       
        turningEncoder = new CANcoder(CANCoderId);

        turningPidController = new PIDController(Constants.ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteDriveEncoderOffset = driveMotor.getPosition().getValue();
    }

    public double getTurningPosition() {
        return turningEncoder.getAbsolutePosition().getValue() - absoluteEncoderOffsetRad;
    }
    
    public double getTurningVelocity() {
        return turningEncoder.getVelocity().getValue();
    }

    public SwerveModuleState getState() {
        // * 10d because getSelectedSensorVelocity() returns ticks/100ms; 
        return new SwerveModuleState(driveMotor.getVelocity().getValue() * 10d * Constants.ModuleConstants.kDriveEncoderTicks2Meter, new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition((driveMotor.getPosition().getValue()-absoluteDriveEncoderOffset) * Constants.ModuleConstants.kDriveEncoderTicks2Meter, getState().angle);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    //private int count = 0;

    @Override
    public void periodic() {

    }

    public double getPosition(){
        return driveMotor.getPosition().getValue();
    }

    public void setMotor(double speed){
        driveMotor.set(speed);
    }

    public double getDrivingTickValues(){
        return (driveMotor.getPosition().getValue()-absoluteDriveEncoderOffset);
    }

    public void setDrivingTickValues(double val){
        driveMotor.setPosition(val);
    }
}
