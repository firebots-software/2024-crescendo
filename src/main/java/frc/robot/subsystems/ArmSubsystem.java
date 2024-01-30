package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX r1, r2, l1, l2;
  private TalonFX master;
  private Encoder mEncoder;
  private ArmFeedforward armff;
  // private TrapezoidProfile profile;
  // private TrapezoidProfile.Constraints tp;

  private MotionMagicConfigs mmc;
  private static ArmSubsystem instance;

  public ArmSubsystem() {
    // tp = new TrapezoidProfile.Constraints(10, 20);
    // profile = new TrapezoidProfile(tp);
    mEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

    Slot0Configs s0c = new Slot0Configs().withKP(0.1).withKI(0).withKD(0);
    armff = new ArmFeedforward(0, 0, 0);
    r1 = new TalonFX(Constants.Arm.R1_PORT);
    r2 = new TalonFX(Constants.Arm.R2_PORT);
    l1 = new TalonFX(Constants.Arm.L1_PORT);
    l2 = new TalonFX(Constants.Arm.L2_PORT);

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
    master.getConfigurator().apply(s0c);

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = 80;
    mmc.MotionMagicAcceleration = 160;
    mmc.MotionMagicJerk = 1600;
    master.getConfigurator().apply(mmc);

    // What ports is the encoder on? They should be two different DIO ports:
    // mEncoder = new Encoder(0, 1);
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private void setPosition(double angleDegrees) {
    MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
    m_request.withFeedForward(armff.calculate(angleDegrees * Math.PI * 2, 0, 0));
    master.setControl(m_request.withPosition(angleDegrees / 360));
  }

  public double getPosition() {
    return master.getPosition().getValue();
  }

  public double determineAngle(Pose2d a, double fkla) {
    return -1;
  }

  public void rotateArmToSpeakerPosition() {
    setPosition(Constants.Arm.ARM_ENCODER_OFFSET + Constants.Arm.SPEAKER_ANGLE);
  }

  public void rotateArmToRestPosition() {
    setPosition(Constants.Arm.ARM_ENCODER_OFFSET);
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
}
