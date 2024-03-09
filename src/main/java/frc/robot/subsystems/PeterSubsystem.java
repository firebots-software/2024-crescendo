package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PeterSubsystem extends SubsystemBase {
  private static PeterSubsystem instance;

  private DigitalInput noteSensor;
  private TalonFX shooterMotorUp, shooterMotorDown;
  private TalonFX preShooterMotor, intakeMotor;

  private MotionMagicConfigs mmcPreShooter;

  public PeterSubsystem() {
    // Initalize shooter
    // Follower f = new Follower(Constants.Intake.SHOOTER_PORT_LEFT, false );
    shooterMotorDown = new TalonFX(Constants.Pooer.SHOOTER.PORT_1, Constants.Pooer.CANBUS_NAME);
    shooterMotorUp = new TalonFX(Constants.Pooer.SHOOTER.PORT_2, Constants.Pooer.CANBUS_NAME);
    shooterMotorUp.setInverted(true);
    // shooterMotorRight.setControl(f);
    Slot0Configs s0c =
        new Slot0Configs().withKP(0.001).withKI(0).withKD(0).withKG(0).withKV(0.2).withKA(0);
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Pooer.SHOOTER.STATOR_CURRENT_LIMIT_AMPS);

    shooterMotorUp.getConfigurator().apply(s0c);
    shooterMotorDown.getConfigurator().apply(s0c);
    shooterMotorUp.getConfigurator().apply(clc);
    shooterMotorDown.getConfigurator().apply(clc);

    // Preshooter
    preShooterMotor = new TalonFX(Constants.Pooer.PRE_SHOOTER_PORT, Constants.Pooer.CANBUS_NAME);
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
                .withStatorCurrentLimit(Constants.Pooer.PRESHOOTER_STATOR_CURRENT_LIMIT_AMPS));
    // Intake
    Slot0Configs intakePid =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    intakeMotor = new TalonFX(Constants.Pooer.INTAKE_MOTOR_PORT, Constants.Pooer.CANBUS_NAME);
    intakeMotor.getConfigurator().apply(intakePid);
    intakeMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Pooer.INTAKE_STATOR_CURRENT_LIMIT_AMPS));
    intakeMotor.setInverted(true);
    noteSensor = new DigitalInput(Constants.Pooer.NOTE_DETECTOR_PORT);
  }

  public static PeterSubsystem getInstance() {
    if (instance == null) {
      instance = new PeterSubsystem();
    }
    return instance;
  }

  // INTAKE FUNCTIONS:
  public void spinUpIntake() {
    runIntakeAtRPS(Constants.Pooer.INTAKE_WHEEL_SPEED_RPS);
  }

  private void runIntakeAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Pooer.INTAKE_GEAR_RATIO);
    m_velocityControl.withFeedForward(0.1);
    intakeMotor.setControl(m_velocityControl);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // SHOOTER FUNCTIONS:
  private void runRightShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Pooer.SHOOTER.GEAR_RATIO);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorUp.setControl(m_velocityControl);
    // shooterMotorUp.getVelocity();
  }

  private void runLeftShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.Pooer.SHOOTER.GEAR_RATIO);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorDown.setControl(m_velocityControl);
  }

  public void stopShooter(boolean forceStop) {
    if (forceStop) {
      runRightShooterAtRPS(0);
      runLeftShooterAtRPS(0);
    } else {
      shooterMotorDown.stopMotor();
      shooterMotorUp.stopMotor();
    }
  }

  public void spinRightShooter() {
    runRightShooterAtRPS(Constants.Pooer.SHOOTER.SPEED_RPS);
  }

  public void spinLeftShooter() {
    runLeftShooterAtRPS(Constants.Pooer.SHOOTER.SPEED_RPS);
  }

  public void stopRightShooter() {
    shooterMotorUp.stopMotor();
  }

  public void stopLeftShooter() {
    shooterMotorDown.stopMotor();
  }

  public void resetPreshooterPosition() {
    preShooterMotor.setPosition(0);
  }

  public void reversePreshooterRotations(double count) {
    preShooterMotor.setControl(new PositionVoltage(-count * Constants.Pooer.PRESHOOTER_GEAR_RATIO));
  }

  public boolean isBackedUp(double count) {
    return Math.abs(
            preShooterMotor.getPosition().getValueAsDouble()
                - (-count * Constants.Pooer.PRESHOOTER_GEAR_RATIO))
        < 0.1;
  }

  public void reverseMechanism() {
    preShooterMotor.setControl(new DutyCycleOut(-0.5));
    shooterMotorDown.setControl(new DutyCycleOut(-0.5));
    shooterMotorUp.setControl(new DutyCycleOut(-0.5));
    intakeMotor.setControl(new DutyCycleOut(-0.5));
  }

  public boolean isShooterReady() {
    return Math.abs(
            shooterMotorDown.getVelocity().getValueAsDouble()
                - (Constants.Pooer.SHOOTER.SPEED_RPS * Constants.Pooer.SHOOTER.GEAR_RATIO))
        < 10;
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
    runPreShooterAtRPS(Constants.Pooer.ROTATIONS_TO_SHOOTER);
  }

  private void runPreShooterAtRPS(double speed) {
    VoltageOut velocityControl = new VoltageOut(Constants.Pooer.PRESHOOTER_WHEEL_VOLTAGE);
    preShooterMotor.setControl(velocityControl);

    // VelocityVoltage m_velocityControl =
    //     new VelocityVoltage(speed * Constants.Peter.PRESHOOTER_GEAR_RATIO);
    // m_velocityControl.withFeedForward(0.1);
    // preShooterMotor.setControl(m_velocityControl);
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
        "Shooter down speed", shooterMotorDown.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter up speed", shooterMotorUp.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter down power", shooterMotorDown.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Shooter up power", shooterMotorUp.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter down current", shooterMotorDown.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter up current", shooterMotorUp.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putString(
        "Command",
        (this.getCurrentCommand() == null
            ? "none"
            : this.getCurrentCommand().getName())); // false = note detected!!
    SmartDashboard.putString(
        "Current commannd PETER:",
        (getCurrentCommand() == null) ? "NULL" : getCurrentCommand().getName());
    periodicSignalLogger();
  }

  public void periodicSignalLogger() {
    SignalLogger.writeBoolean("Note Detected", notePresent());
    SignalLogger.writeBoolean("Shooter Ready", isShooterReady());
    SignalLogger.writeDouble(
        "Shooter down speed", shooterMotorDown.getVelocity().getValueAsDouble());
    SignalLogger.writeDouble("Shooter up speed", shooterMotorUp.getVelocity().getValueAsDouble());
    SignalLogger.writeDouble(
        "Shooter down power", shooterMotorDown.getDutyCycle().getValueAsDouble());
    SignalLogger.writeDouble("Shooter up power", shooterMotorUp.getDutyCycle().getValueAsDouble());
    SignalLogger.writeDouble(
        "Shooter down current", shooterMotorDown.getStatorCurrent().getValueAsDouble());
    SignalLogger.writeDouble(
        "Shooter up current", shooterMotorUp.getStatorCurrent().getValueAsDouble());
  }
  // public void runShooter(int i) {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'runShooter'");
  // }
}
