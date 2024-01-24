package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private StatusSignal<Double> preShooterPosition;

  private MotionMagicConfigs mmcPreShooter;
  private TrapezoidProfile profile;
  private TrapezoidProfile.Constraints tp;

  public PeterSubsystem() {
    tp = new TrapezoidProfile.Constraints(10, 20);
    profile = new TrapezoidProfile(tp);

    // Initalize shooter
    Follower f = new Follower(Constants.Intake.SHOOTER_PORT_LEFT, false);
    shooterMotorLeft = new TalonFX(Constants.Intake.SHOOTER_PORT_LEFT);
    shooterMotorRight = new TalonFX(Constants.Intake.SHOOTER_PORT_RIGHT);
    shooterMotorRight.setInverted(true);
    shooterMotorRight.setControl(f);
    shooterMotorMaster = shooterMotorLeft;
    Slot0Configs s0c =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    shooterMotorMaster.getConfigurator().apply(s0c);

    // Preshooter
    preShooterMotor = new TalonFX(Constants.Intake.PRE_SHOOTER_PORT);
    mmcPreShooter = new MotionMagicConfigs();
    mmcPreShooter.MotionMagicCruiseVelocity = 80;
    mmcPreShooter.MotionMagicAcceleration = 160;
    mmcPreShooter.MotionMagicJerk = 1600;
    preShooterMotor.getConfigurator().apply(mmcPreShooter);

    // Intake
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);

    noteSensor = new DigitalInput(Constants.Intake.NOTE_DETECTOR_PORT);

    rollerMotorRequest = new DutyCycleOut(0.0);
    preShooterPosition = preShooterMotor.getPosition();
  }

  public static PeterSubsystem getInstance() {
    if (instance == null) {
      instance = new PeterSubsystem();
    }
    return instance;
  }

  public void runIntake(
      double speed) { // TODO: We have convert a distance to a speed. We take in velocity in rps
    VelocityVoltage m_velocityControl = new VelocityVoltage(speed);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorMaster.setControl(m_velocityControl);
  }

  public void runShooter(
      double speed) { // TODO: We have convert a distance to a speed. We take in velocity in rps
    VelocityVoltage m_velocityControl = new VelocityVoltage(speed);
    m_velocityControl.withFeedForward(0.1);
    shooterMotorMaster.setControl(m_velocityControl);
  }

  public boolean notePresent() {
    return noteSensor.get(); // true = note present
  }

  public void moveNoteToShooter(double speed) {
    MotionMagicVoltage m_request = new MotionMagicVoltage(preShooterMotor.getPosition().getValue());
    preShooterMotor.setControl(
        m_request.withPosition(
            preShooterMotor.getPosition().getValue()
                + Constants.Intake.ROTATIONS_TO_SHOOTER)); // rotate 5 more rotationes
  }

  public double getPreShooterPosition() {
    return preShooterPosition.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Detected", notePresent()); // false = note detected!!
  }
}
