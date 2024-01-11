package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveSubsystem extends SubsystemBase{
    private TalonFX turningMotor;
    private TalonFX driveMotor;
    private CANcoder coder;
    
    public SwerveSubsystem (int turningMotorPort, int driveMotorPort, int CANcoderPort){
        this.turningMotor = new TalonFX(turningMotorPort);
        this.driveMotor = new TalonFX(driveMotorPort);
        this.coder = new CANcoder(CANcoderPort);
    }

    public void setState(SwerveModuleState sms){
        Rotation2d angle = sms.angle;
        double speed = sms.speedMetersPerSecond;
        turningMotor.setPosition(ticks(angle.getRadians()));
        driveMotor.set(speed);
    }

    private double ticks(double radians){
        return radians * 4096;
    }
}
