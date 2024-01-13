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

public class Intake extends SubsystemBase {

   // private static final int MAX_DISTANCE = 4048;//
    private static Intake instance;
    private final DutyCycleOut rollerMotorRequest = new DutyCycleOut(0.0);
    private DigitalInput input;
    public TalonFX rollerMotor;


    private Intake() {
        rollerMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
        input = new DigitalInput(Constants.Intake.NOTE_DETECTOR_PORT);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void runIntake(double speed) {
        rollerMotor.setControl(rollerMotorRequest.withOutput(speed));
    }

    public void rotateArmToRestPosition() {

    }


    public boolean notePresent() {
        return input.get(); //true = note present
    }  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
