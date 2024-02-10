package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestEncoderSubsystem extends SubsystemBase {
  private TalonFX r1, r2, l1, l2;
  private TalonFX master;
  private DutyCycleEncoder revEncoder;
  private ArmFeedforward armff;
  // private TrapezoidProfile profile;
  // private TrapezoidProfile.Constraints tp;

  private MotionMagicConfigs mmc;
  private static TestEncoderSubsystem instance;
  private double targetPos;

  public TestEncoderSubsystem() {
    // tp = new TrapezoidProfile.Constraints(10, 20);
    // profile = new TrapezoidProfile(tp);
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

    Slot0Configs s0c = new Slot0Configs().withKP(37).withKI(0).withKD(0);
    armff = new ArmFeedforward(0.1, 0.1, 0.1);
    r1 = new TalonFX(Constants.Swerve.FRONT_RIGHT.SteerMotorId, Constants.Swerve.CANBUS_NAME);
    r2 = new TalonFX(Constants.Swerve.BACK_RIGHT.SteerMotorId, Constants.Swerve.CANBUS_NAME);
    l1 = new TalonFX(Constants.Swerve.FRONT_LEFT.SteerMotorId, Constants.Swerve.CANBUS_NAME);
    l2 = new TalonFX(Constants.Swerve.BACK_LEFT.SteerMotorId, Constants.Swerve.CANBUS_NAME);

    Follower f = new Follower(Constants.Arm.R1_PORT, false);
    r2.setControl(f);
    l1.setInverted(true);
    l1.setControl(f);
    l2.setInverted(true);
    l2.setControl(f);

    r1.getConfigurator().apply(clc);
    r2.getConfigurator().apply(clc);
    l1.getConfigurator().apply(clc);
    l2.getConfigurator().apply(clc);

    master = r1;
    TalonFXConfigurator masterConfigurator = master.getConfigurator();
    masterConfigurator.apply(s0c);
    masterConfigurator.apply(
        new FeedbackConfigs());

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = 80;
    mmc.MotionMagicAcceleration = 160;
    mmc.MotionMagicJerk = 1600;
    masterConfigurator.apply(mmc);

    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
    revEncoder.setPositionOffset(Constants.Arm.ARM_ENCODER_OFFSET);
    master.setPosition((revEncoder.getAbsolutePosition() - 0.1) * Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);
    
    targetPos = Constants.Arm.DEFAULT_ARM_ANGLE;
  }

  // private TalonFXConfigurator apply(Slot0Configs s0c) {
  //   // TODO Auto-generated method stub the method wasn't being used so commented our for now
  //   throw new UnsupportedOperationException("Unimplemented method 'apply'");
  // }

  public static TestEncoderSubsystem getInstance() {
    if (instance == null) {
      instance = new TestEncoderSubsystem();
    }
    return instance;
  }

  private void setPosition(double angleDegrees) {
    MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
    master.setControl(
        m_request
            .withPosition(angleDegrees / 360)
            .withFeedForward(armff.calculate(getPosition() * Math.PI * 2 / 360, 0)));
    // input is in rotations
  }

  public double getPosition() {
    return master.getPosition().getValue();
  }

  public void setTargetPosition(double angleDegrees) {
    targetPos = angleDegrees;
  }

  public double determineAngle(Pose2d a, double fkla) {
    return -1;
  }

  public void rotateToSpeakerPosition() {
    setTargetPosition(Constants.Swerve.FRONT_RIGHT.CANcoderOffset + 60.0);
  }

  public void rotateToResetPosition() {
    setTargetPosition(Constants.Swerve.FRONT_RIGHT.CANcoderOffset);
  }

  // public void toPosition() {
  //   // Magic Motion:
  //   MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
  //   master.setControl(m_request.withPosition(setPos));

  // // Trapizoidal Motion:
  // TrapezoidProfile.State setPoint = new TrapezoidProfile.State(setPos, 0);
  // TrapezoidProfile.State currentPoint = new
  // TrapezoidProfile.State(master.getPosition().getValue(),master.getVelocity().getValue());

  // setPoint = profile.calculate(profile.totalTime(), currentPoint, setPoint);
  // PositionDutyCycle m_positionControl = new
  // PositionDutyCycle(setPoint.position);
  // m_positionControl.Position = setPoint.position;
  // m_positionControl.Velocity = setPoint.velocity;
  // master.setControl(m_positionControl);
  // }

  @Override
  public void periodic() {
    setPosition(targetPos);
    SmartDashboard.putNumber("Front right motor pos: ", getPosition());
    SmartDashboard.putBoolean("Front right sensor overflow: ", master.getFault_RemoteSensorPosOverflow().getValue());
    SmartDashboard.putNumber("Front right set speed: ", master.get());
  }
}
