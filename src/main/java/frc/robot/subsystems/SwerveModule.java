package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule extends SubsystemBase {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final CANcoder turningEncoder;

    private final PIDController turningPidController;

    private final double absoluteEncoderOffsetRotations;
    private final double absoluteDriveEncoderOffset;

    public SwerveModule(int driveMotorId, int turningMotorId, int CANCoderId, boolean driveMotorReversed, boolean turningMotorReversed, double absoluteEncoderOffsetRotations) {

        this.absoluteEncoderOffsetRotations = absoluteEncoderOffsetRotations;

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
       
        turningEncoder = new CANcoder(CANCoderId);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-0.5, 0.5);

        absoluteDriveEncoderOffset = driveMotor.getPosition().getValue(); // getselected sensorposition
    }

    public double getTurningPosition() {
        return turningEncoder.getAbsolutePosition().getValue() - absoluteEncoderOffsetRotations;
    }
    
    public double getTurningVelocity() {
        return turningEncoder.getVelocity().getValue();
    }

    public SwerveModuleState getState() {
        // * 10d because getSelectedSensorVelocity() returns ticks/100ms; 
        return new SwerveModuleState(driveMotor.getVelocity().getValue() / (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()/(2.0*Math.PI)));
    }

    public void setTurnState(double toRad) {
        turningMotor.set(turningPidController.calculate(getTurningPosition(), toRad/(2.0*Math.PI)));
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
    /*
        count++;
        if (count % 150 == 0) {
            System.out.println("actual m/s: " + getState().speedMetersPerSecond + "\n.\texpected m/s: " + DEBUG_lastms);
            System.out.println("power: " + DEBUG_lastms / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            //System.out.println("distance m: " + getModulePosition().distanceMeters);
            count = 0;
        }
    */
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

