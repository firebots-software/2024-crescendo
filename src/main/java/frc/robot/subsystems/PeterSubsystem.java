package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PeterSubsystem extends SubsystemBase {
  private static PeterSubsystem instance;

  private DigitalInput noteSensor;
  private TalonFX shooterMotorRight, shooterMotorLeft;
  private TalonFX preShooterMotor, intakeMotor;
  private StatusSignal<Double> preShooterPosition;

  private MotionMagicConfigs mmcPreShooter;

  public PeterSubsystem() {
    // Initalize shooter
    // Follower f = new Follower(Constants.Intake.SHOOTER_PORT_LEFT, false );
    shooterMotorLeft =
        new TalonFX(Constants.Intake.SHOOTER_PORT_LEFT, Constants.Intake.CANBUS_NAME);
    shooterMotorRight =
        new TalonFX(Constants.Intake.SHOOTER_PORT_RIGHT, Constants.Intake.CANBUS_NAME);
    shooterMotorRight.setInverted(true);
    // shooterMotorRight.setControl(f);
    Slot0Configs s0c =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    shooterMotorRight.getConfigurator().apply(s0c);
    shooterMotorLeft.getConfigurator().apply(s0c);

    // Preshooter
    preShooterMotor = new TalonFX(Constants.Intake.PRE_SHOOTER_PORT, Constants.Intake.CANBUS_NAME);
    mmcPreShooter = new MotionMagicConfigs();
    mmcPreShooter.MotionMagicCruiseVelocity = 80;
    mmcPreShooter.MotionMagicAcceleration = 160;
    mmcPreShooter.MotionMagicJerk = 1600;
    preShooterMotor.getConfigurator().apply(mmcPreShooter);

    // Intake
    Slot0Configs intakePid =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT, Constants.Intake.CANBUS_NAME);
    intakeMotor.getConfigurator().apply(intakePid);
    intakeMotor.setInverted(true);
    // noteSensor = new DigitalInput(Constants.Intake.NOTE_DETECTOR_PORT);

    preShooterPosition = preShooterMotor.getPosition();
  }

  public static PeterSubsystem getInstance() {
    if (instance == null) {
      instance = new PeterSubsystem();
    }
    return instance;
  }

  // INTAKE FUNCTIONS:
  public void spinUpIntake() {
    runIntakeAtRPS(Constants.Intake.INTAKE_WHEEL_SPEED_RPS);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  private void runIntakeAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(Constants.Intake.INTAKE_WHEEL_SPEED_RPS);
    m_velocityControl.withFeedForward(0.1);
    intakeMotor.setControl(m_velocityControl);
  }

  // SHOOTER FUNCTIONS:
  public void spinUpShooter() {
    // runShooterAtRPS(Constants.Intake.SHOOT_WHEEL_SPEED_RPS);
    runRightShooterAtRPS(Constants.Intake.SHOOT_WHEEL_SPEED_RPS);
    runLeftShooterAtRPS(Constants.Intake.SHOOT_WHEEL_SPEED_RPS);
  }

  public void stopShooter() {
    shooterMotorLeft.stopMotor();
    shooterMotorRight.stopMotor();
  }

  public void spinRightShooter() {
    runRightShooterAtRPS(Constants.Intake.SHOOT_WHEEL_SPEED_RPS);
  }

  public void spinLeftShooter() {
    runLeftShooterAtRPS(Constants.Intake.SHOOT_WHEEL_SPEED_RPS);
  }

  public void stopRightShooter() {
    shooterMotorRight.stopMotor();
  }

  public void stopLeftShooter() {
    shooterMotorLeft.stopMotor();
  }

  private void runRightShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl = new VelocityVoltage(speed);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorRight.setControl(m_velocityControl);
  }

  private void runLeftShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl = new VelocityVoltage(speed);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorLeft.setControl(m_velocityControl);
  }

  private void runShooterAtRPS(double speed) {
    // VelocityVoltage m_velocityControl = new VelocityVoltage(speed);
    // m_velocityControl.withFeedForward(0.1);
    // shooterMotorRight.setControl(m_velocityControl);
    // shooterMotorLeft.setControl(m_velocityControl);
    runRightShooterAtRPS(speed);
    runLeftShooterAtRPS(speed);
  }

  public boolean isShooterReady() {
    if (Math.abs(shooterMotorLeft.getVelocity().getValue() - Constants.Intake.SHOOT_WHEEL_SPEED_RPS)
        < 0.001) {
      return true;
    }
    return false;
  }

  // SENSOR FUNCTIONS:
  public static boolean noteStaticPresent() {
    if (instance != null) {
      return instance.notePresent(); // true = note present
    } else {
      throw new NullPointerException(
          "In order to use the Arm subsystem, we need to access the sensor from shooter. Please initalize the PeterSubsystem before using an Arm Command");
    }
  }

  public boolean notePresent() {
    return false; // noteSensor.get();
  }

  // PRE-SHOOTER FUNCTIONS:
  public void moveNoteToShooter() {
    movePreShooterMotorPosition(Constants.Intake.ROTATIONS_TO_SHOOTER); // 5 rotations
  }

  public void stopPreShooterMotor() {
    movePreShooterMotorPosition(0);
  }

  public void movePreShooterMotorPosition(double position) { // rotates by `position` more rotations
    MotionMagicVoltage m_request = new MotionMagicVoltage(preShooterMotor.getPosition().getValue());
    preShooterMotor.setControl(
        m_request.withPosition(
            preShooterMotor.getPosition().getValue() + position)); // rotate 5 more rotations
  }

  public double getPreShooterPosition() {
    return preShooterPosition.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Note Detected", notePresent()); // false = note detected!!
  }

  public void runShooter(double i) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runShooter'");
  }
}
