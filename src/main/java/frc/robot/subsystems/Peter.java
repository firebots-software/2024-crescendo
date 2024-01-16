package frc.robot.subsystems;

import javax.xml.validation.SchemaFactory;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix6.sensors.CANCoder;
import com.ctre.phoenix6.sensors.CANCoderConfiguration;

public class Peter extends SubsystemBase {
    private final double absoluteEncoderOffsetRad;
    private final double absoluteDriveEncoderOffset;
    private final PIDController turningPidController;


   // private static final int MAX_DISTANCE = 4048;//
    private static Peter instance;

    //private final PositionDutyCycle v;

    private final DutyCycleOut rollerMotorRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut preShooterMotorRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut ShooterMotorRequest = new DutyCycleOut(0.0);
    private final PositionDutyCycle preShooterDistanceMotorRequest = new PositionDutyCycle(0,0, false, 0, 0, false, false, false);
    private DigitalInput input;
    public TalonFX rollerMotor, preShooterMotor, shooterMotor;
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;

    public Peter() {
        rollerMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
        input = new DigitalInput(Constants.Intake.NOTE_DETECTOR_PORT);
        preShooterMotor = new TalonFX(Constants.Intake.PRE_SHOOTER_PORT);
        shooterMotor = new TalonFX(Constants.Intake.SHOOTER_PORT);
        

    }

    public static Peter getInstance() {
        if (instance == null) {
            instance = new Peter();
        }
        return instance;
    }

    public void runIntake(double speed) {
        rollerMotor.setControl(rollerMotorRequest.withOutput(speed));
        
    }


    public void runShooter(double speed){
        shooterMotor.setControl(ShooterMotorRequest.withOutput(speed));
    }

    public void rotateArmToRestPosition() {

    }


    public boolean notePresent() {
        return input.get(); //true = note present
    }  

    public void runPreShooter(double speed) {
        preShooterMotor.setControl(ShooterMotorRequest.withOutput(speed));
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Note Detected", input.get());
    }


}
