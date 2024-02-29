package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    shooterMotorLeft = new TalonFX(Constants.Peter.SHOOTER_PORT_LEFT, Constants.Peter.CANBUS_NAME);
    shooterMotorRight =
        new TalonFX(Constants.Peter.SHOOTER_PORT_RIGHT, Constants.Peter.CANBUS_NAME);
    shooterMotorRight.setInverted(true);
    // shooterMotorRight.setControl(f);
    Slot0Configs s0c =
        new Slot0Configs().withKP(0.001).withKI(0).withKD(0).withKG(0).withKV(0.2).withKA(0);

    shooterMotorRight.getConfigurator().apply(s0c);
    shooterMotorLeft.getConfigurator().apply(s0c);
    shooterMotorLeft
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Peter.SHOOTER_STATOR_CURRENT_LIMIT_AMPS));
    shooterMotorRight
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Peter.SHOOTER_STATOR_CURRENT_LIMIT_AMPS));

    // Preshooter
    preShooterMotor = new TalonFX(Constants.Peter.PRE_SHOOTER_PORT, Constants.Peter.CANBUS_NAME);
    mmcPreShooter = new MotionMagicConfigs();
    mmcPreShooter.MotionMagicCruiseVelocity = 80;
    mmcPreShooter.MotionMagicAcceleration = 160;
    mmcPreShooter.MotionMagicJerk = 1600;
    preShooterMotor.getConfigurator().apply(mmcPreShooter);

    Slot0Configs preshooterPID = new Slot0Configs().withKP(3).withKV(1);
    preShooterMotor.getConfigurator().apply(preshooterPID);
    preShooterMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Peter.PRESHOOTER_STATOR_CURRENT_LIMIT_AMPS));
    // Intake
    Slot0Configs intakePid =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    intakeMotor = new TalonFX(Constants.Peter.INTAKE_MOTOR_PORT, Constants.Peter.CANBUS_NAME);
    intakeMotor.getConfigurator().apply(intakePid);
    intakeMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Peter.INTAKE_STATOR_CURRENT_LIMIT_AMPS));
    intakeMotor.setInverted(true);
    noteSensor = new DigitalInput(Constants.Peter.NOTE_DETECTOR_PORT);

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
    runIntakeAtRPS(Constants.Peter.INTAKE_WHEEL_SPEED_RPS);
  }

  private void runIntakeAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Peter.INTAKE_GEAR_RATIO);
    m_velocityControl.withFeedForward(0.1);
    intakeMotor.setControl(m_velocityControl);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // SHOOTER FUNCTIONS:
  private void runRightShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Peter.SHOOTER_WHEELS_GEAR_RATIOS);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorRight.setControl(m_velocityControl);
    shooterMotorRight.getVelocity();
  }

  private void runLeftShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Peter.SHOOTER_WHEELS_GEAR_RATIOS);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorLeft.setControl(m_velocityControl);
  }

  public void stopShooter(boolean forceStop) {
    if (forceStop) {
      runRightShooterAtRPS(0);
      runLeftShooterAtRPS(0);
    } else {
      shooterMotorLeft.stopMotor();
      shooterMotorRight.stopMotor();
    }
  }

  public void spinRightShooter() {
    runRightShooterAtRPS(Constants.Peter.SHOOT_WHEEL_SPEED_RPS);
  }

  public void spinLeftShooter() {
    runLeftShooterAtRPS(Constants.Peter.SHOOT_WHEEL_SPEED_RPS);
  }

  public void stopRightShooter() {
    shooterMotorRight.stopMotor();
  }

  public void stopLeftShooter() {
    shooterMotorLeft.stopMotor();
  }

  public void resetPreshooterPosition() {
    preShooterMotor.setPosition(0);
  }

  public void reversePreshooterRotations(double count) {
    preShooterMotor.setControl(new PositionVoltage(-count * Constants.Peter.PRESHOOTER_GEAR_RATIO));
  }

  public boolean isBackedUp(double count) {
    return Math.abs(
            preShooterMotor.getPosition().getValueAsDouble()
                - (-count * Constants.Peter.PRESHOOTER_GEAR_RATIO))
        < 0.1;
  }

  public void reverseMechanism() {
    preShooterMotor.setControl(new DutyCycleOut(-0.5));
    shooterMotorLeft.setControl(new DutyCycleOut(-0.5));
    shooterMotorRight.setControl(new DutyCycleOut(-0.5));
    intakeMotor.setControl(new DutyCycleOut(-0.5));
  }

  public boolean isShooterReady() {
    if (Math.abs(shooterMotorLeft.getVelocity().getValue() - Constants.Peter.SHOOT_WHEEL_SPEED_RPS)
        < 0.001) {
      return true;
    }
    if (Math.abs(
            shooterMotorLeft.getVelocity().getValueAsDouble()
                - (Constants.Peter.SHOOT_WHEEL_SPEED_RPS
                    * Constants.Peter.SHOOTER_WHEELS_GEAR_RATIOS))
        < 10) {
      return true;
    }
    return false;
  }

  /* private void runShooterAtRPS(double speed) {
    runRightShooterAtRPS(speed);
    runLeftShooterAtRPS(speed);
    // VelocityVoltage m_velocityControl = new VelocityVoltage(speed);
    // m_velocityControl.withFeedForward(0.1);
    // shooterMotorRight.setControl(m_velocityControl);
    // shooterMotorLeft.setControl(m_velocityControl);

  } */

  // SENSOR FUNCTIONS:
  public boolean notePresent() {
    return !noteSensor.get();
  }

  // PRE-SHOOTER FUNCTIONS:

  public void spinUpPreShooter() {
    runPreShooterAtRPS(Constants.Peter.ROTATIONS_TO_SHOOTER);
  }

  private void runPreShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Peter.PRESHOOTER_GEAR_RATIO);
    m_velocityControl.withFeedForward(0.1);
    preShooterMotor.setControl(m_velocityControl);
  }

  public void stopPreShooterMotor() {
    preShooterMotor.stopMotor();
  }

  /* public void moveNoteToShooter() {
    movePreShooterMotorPosition(
        Constants.Peter.ROTATIONS_TO_SHOOTER
            * Constants.Peter.PRESHOOTER_GEAR_RATIO); // 5 rotations
  }*/

  /* public void movePreShooterMotorPosition(double position) { // rotates by `position` more rotations
    MotionMagicVoltage m_request = new MotionMagicVoltage(preShooterMotor.getPosition().getValue());
    preShooterMotor.setControl(
        m_request.withPosition(
            preShooterMotor.getPosition().getValue() + position)); // rotate 5 more rotations
  } */

  /* public double getPreShooterPosition() {
    return preShooterPosition.getValue();
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Detected", notePresent()); // false = note detected!!
    SmartDashboard.putBoolean("Shooter Ready", isShooterReady());
    SmartDashboard.putNumber(
        "Shooter left speed", shooterMotorLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter right speed", shooterMotorRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter left power", shooterMotorLeft.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter right power", shooterMotorRight.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter left current", shooterMotorLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter right current", shooterMotorRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putString(
        "Command",
        (this.getCurrentCommand() == null
            ? "none"
            : this.getCurrentCommand().getName())); // false = note detected!!
    SmartDashboard.putString(
        "Current commannd PETER:",
        (getCurrentCommand() == null) ? "NULL" : getCurrentCommand().getName());
  }

  public void runShooter(int i) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runShooter'");
  }
}
