package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PeterSubsystem extends SubsystemBase {
  // private static final int MAX_DISTANCE = 4048;//
  private static PeterSubsystem instance;

  // private final PositionDutyCycle v;
  private final DutyCycleOut rollerMotorRequest;

  private DigitalInput noteSensor;
  public TalonFX shooterMotorRight, shooterMotorLeft, shooterMotorMaster;
  public TalonFX preShooterMotor, intakeMotor;
  private StatusSignal<Double> position;

  public PeterSubsystem() {
    Follower f = new Follower(Constants.Intake.SHOOTER_PORT_LEFT, false);
    shooterMotorLeft = new TalonFX(Constants.Intake.SHOOTER_PORT_LEFT);
    shooterMotorRight = new TalonFX(Constants.Intake.SHOOTER_PORT_RIGHT);
    shooterMotorRight.setInverted(true);
    shooterMotorRight.setControl(f);

    shooterMotorMaster = shooterMotorLeft;
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
    preShooterMotor = new TalonFX(Constants.Intake.PRE_SHOOTER_PORT);
    noteSensor = new DigitalInput(Constants.Intake.NOTE_DETECTOR_PORT);

    rollerMotorRequest = new DutyCycleOut(0.0);
    Slot0Configs s0c =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    preShooterMotor.getConfigurator().apply(s0c);
    position = preShooterMotor.getPosition();
  }

  public static PeterSubsystem getInstance() {
    if (instance == null) {
      instance = new PeterSubsystem();
    }
    return instance;
  }

  public void runIntake(double speed) {
    intakeMotor.setControl(rollerMotorRequest.withOutput(speed));
  }

  public void runShooter(double speed) {
    shooterMotorMaster.set(speed);
  }

  public boolean notePresent() {
    return noteSensor.get(); // true = note present
  }

  public void runPreShooter(double speed) {
    preShooterMotor.set(speed);
  }

  public double getPreShooterPosition() {
    return position.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Detected", noteSensor.get()); // false = note detected!!
  }
}
