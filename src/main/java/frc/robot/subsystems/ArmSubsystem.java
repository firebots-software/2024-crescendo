package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX r1, r2, l1, l2;
  private TalonFX master;
  private CANcoder absoluteEncoder;
  private ArmFeedforward armff;

  private MotionMagicConfigs mmc;
  private static ArmSubsystem instance;
  private double targetPos;

  public ArmSubsystem() {
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

    Slot0Configs s0c = new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    armff = new ArmFeedforward(0, 0, 0);
    r1 = new TalonFX(Constants.Arm.R1_PORT, Constants.Swerve.CANBUS_NAME);
    r2 = new TalonFX(Constants.Arm.R2_PORT, Constants.Swerve.CANBUS_NAME);
    l1 = new TalonFX(Constants.Arm.L1_PORT, Constants.Swerve.CANBUS_NAME);
    l2 = new TalonFX(Constants.Arm.L2_PORT, Constants.Swerve.CANBUS_NAME);

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
        new FeedbackConfigs().withFeedbackRemoteSensorID(Constants.Arm.ENCODER_PORT));

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = 80;
    mmc.MotionMagicAcceleration = 160;
    mmc.MotionMagicJerk = 1600;
    master.getConfigurator().apply(mmc);

    absoluteEncoder = new CANcoder(Constants.Arm.ENCODER_PORT);

    targetPos = Constants.Arm.DEFAULT_ARM_ANGLE;
    //Constants.Arm.ARM_ENCODER_OFFSET = master.getPosition().getValue();
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private void setPosition(double angleDegrees) {
    MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
    master.setControl(
        m_request
            .withPosition(angleDegrees / 360)
            .withEnableFOC(true)
            .withFeedForward(armff.calculate(getPosition() * Math.PI * 2 / 360, 0)));
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

  public void rotateArmToSpeakerPosition() {
    setTargetPosition(Constants.Arm.ARM_ENCODER_OFFSET + Constants.Arm.SPEAKER_ANGLE);
  }

  public void rotateArmToRestPosition() {
    setTargetPosition(Constants.Arm.ARM_ENCODER_OFFSET);
  }

  // public void toPosition() {
  //   // Magic Motion:
  //   MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
  //   master.setControl(m_request.withPosition(setPos));

  // Trapizoidal Motion:
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
  }
}
