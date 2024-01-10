package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static Intake instance;
    private AnalogInput input;
    public TalonFX rollerMotor;

    private Intake() {
        rollerMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void runIntake(double speed) {
        rollerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopIntakeWhenNoteSeen() {
       if (input.getVoltage() < 1017) {
            runIntake(0);
       }
    }

    public void stopIntakeWhenNoteGone() 
    {
        while(input.getVoltage() > 1017) {
            runIntake(0);
        }
     }

     @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
