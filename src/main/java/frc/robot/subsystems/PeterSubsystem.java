package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PeterSubsystem extends SubsystemBase {
  // private static final int MAX_DISTANCE = 4048;//
  private static PeterSubsystem instance;

  // private final PositionDutyCycle v;
  private final DutyCycleOut rollerMotorRequest = new DutyCycleOut(0.0);
  private final PositionVoltage preShooterMotorPIDRequest = new PositionVoltage(0.0);
  private final DutyCycleOut ShooterMotorRequest = new DutyCycleOut(0.0);
  private final double INTAKE_ANGLE = 0; // subject to change
  private final double AMP_ANGLE = 100; // subject to change
  private DigitalInput input;
  public TalonFX rollerMotor, preShooterMotor, shooterMotor;
  private StatusSignal<Double> position;

  public PeterSubsystem() {
    rollerMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
    input = new DigitalInput(Constants.Intake.NOTE_DETECTOR_PORT);
    preShooterMotor = new TalonFX(Constants.Intake.PRE_SHOOTER_PORT);
    shooterMotor = new TalonFX(Constants.Intake.SHOOTER_PORT);
    Slot0Configs preShooterMotorSlot = new Slot0Configs().withKP(2).withKI(0).withKD(0);
    preShooterMotor.getConfigurator().apply(preShooterMotorSlot);
    position = preShooterMotor.getPosition();
  }

  public static PeterSubsystem getInstance() {
    if (instance == null) {
      instance = new PeterSubsystem();
    }
    return instance;
  }

  public void runIntake(double speed) {
    rollerMotor.setControl(rollerMotorRequest.withOutput(speed));
  }

  public void runShooter(double speed) {
    shooterMotor.setControl(ShooterMotorRequest.withOutput(speed));
  }

  public boolean notePresent() {
    return input.get(); // true = note present
  }

  public void runPreShooter(double speed) {
    preShooterMotor.setControl(ShooterMotorRequest.withOutput(speed));
  }

  public double getPreShooterPosition() {
    return position.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Detected", input.get()); // false = note detected!!
  }
}
