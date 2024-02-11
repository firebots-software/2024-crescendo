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

  private MotionMagicConfigs mmc;
  private static TestEncoderSubsystem instance;
  private double targetDegrees;

  public TestEncoderSubsystem() {
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

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = 80;
    mmc.MotionMagicAcceleration = 160;
    mmc.MotionMagicJerk = 1600;
    masterConfigurator.apply(mmc);

    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
    revEncoder.setPositionOffset(Constants.Arm.ARM_ENCODER_OFFSET);

    master.setPosition((revEncoder.getAbsolutePosition() - 0.1) * Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);
    
    targetDegrees = Constants.Arm.DEFAULT_ARM_ANGLE;
  }

  public static TestEncoderSubsystem getInstance() {
    if (instance == null) {
      instance = new TestEncoderSubsystem();
    }
    return instance;
  }

  private void setPosition(double angleDegrees) {
    master.setControl(new MotionMagicVoltage(Constants.Swerve.STEER_GEAR_RATIO * angleDegrees / 360d));
            //.withFeedForward(armff.calculate(getPosRotations() * Math.PI * 2 / 360, 0)));
    // input is in rotations
  }

  public double getPosRotations() {
    return master.getPosition().getValue() / Constants.Swerve.STEER_GEAR_RATIO;
  }

  public void setTargetDegrees(double angleDegrees) {
    targetDegrees = angleDegrees;
  }

  public double determineAngle(Pose2d a, double fkla) {
    return -1;
  }

  public void rotateToSpeakerPosition() {
    setTargetDegrees(Constants.Swerve.FRONT_RIGHT.CANcoderOffset + 60.0);
  }

  public void rotateToResetPosition() {
    setTargetDegrees(Constants.Swerve.FRONT_RIGHT.CANcoderOffset);
  }

  @Override
  public void periodic() {
    setPosition(targetDegrees);
    SmartDashboard.putNumber("Front right motor pos: ", getPosRotations());
    //SmartDashboard.putBoolean("Front right sensor overflow: ", master.getFault_RemoteSensorPosOverflow().getValue());
    SmartDashboard.putNumber("Front right set speed: ", master.get());
  }
}
