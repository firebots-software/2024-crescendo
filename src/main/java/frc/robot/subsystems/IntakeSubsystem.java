package frc.robot.subsystems;

import javax.xml.validation.SchemaFactory;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

   // private static final int MAX_DISTANCE = 4048;//
    private static IntakeSubsystem instance;
    private final DutyCycleOut rollerMotorRequest = new DutyCycleOut(0.0);
    private DigitalInput input;
    public TalonFX rollerMotor;


    private IntakeSubsystem() {
        rollerMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public void runIntake(double speed) {
        rollerMotor.setControl(rollerMotorRequest.withOutput(speed));
    }

    public void rotateArmToRestPosition() {

    }


    public boolean notePresent() {
        return input.get(); //> MAX_DISTANCE;// 
    }  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
