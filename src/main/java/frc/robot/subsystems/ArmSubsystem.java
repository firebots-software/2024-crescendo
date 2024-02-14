package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX r1, r2, l1, l2;
  private TalonFX master;
  private DutyCycleEncoder revEncoder;
  private ArmFeedforward armff;
  private MotionMagicConfigs mmc;
  private static ArmSubsystem instance;
  
  private double targetDegrees;
  private double integratedArmEncoderOffset;

  public ArmSubsystem() {
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

    Slot0Configs s0c = new Slot0Configs().withKP(37).withKI(0).withKD(0);
    armff = new ArmFeedforward(0.1, 0.1, 0.1);
    r1 = new TalonFX(Constants.Arm.RT_PORT, Constants.Swerve.CANBUS_NAME);
    r2 = new TalonFX(Constants.Arm.RB_PORT, Constants.Swerve.CANBUS_NAME);
    l1 = new TalonFX(Constants.Arm.LT_PORT, Constants.Swerve.CANBUS_NAME);
    l2 = new TalonFX(Constants.Arm.LB_PORT, Constants.Swerve.CANBUS_NAME);

    Follower f = new Follower(Constants.Arm.RT_PORT, false);
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

    // ==== EXPLANATION: ====
    // getAbsolutePosition(): Absolute Encoder's current reading
    // ABSOLUTE_ENCODER_HORIZONTAL: What the Absolute Encoder reads at horizontal
    // ABSOLUTE_HORIZONTAL_OFFSET: Number of rotations to offset angle count from horizontal, to avoid "pretzeling" or slamming into robot
    // What this next line does:
    // Uses the Absolute Encoder to set the position of the Master motor, so that when the Master motor reads 0 rotations,
    // it represents the arm being at horizontal. After this next line runs, the Master motor's encoder reading can be used
    // like expected, so you simply need to divide its reading by INTEGRATED_ARM_CONVERSION_FACTOR to get the arm's angle in rotations.
    // ======================
    master.setPosition((getAbsolutePosition()-Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL-Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET)*Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);

    targetDegrees = Constants.Arm.DEFAULT_ARM_ANGLE;
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private void setPosition(double angleDegrees) {
    master.setControl(new MotionMagicVoltage(
      integratedArmEncoderOffset + Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR * angleDegrees / 360d));
    // .withFeedForward(armff.calculate(getPosRotations() * Math.PI * 2 / 360, 0)));
    // input is in rotations
  }

  private double getAbsolutePosition() {
    return MathUtil.clamp(revEncoder.getAbsolutePosition() + Constants.Arm.ARM_ENCODER_OFFSET, 0d, 1d);
  }

  public double getPosRotations() {
    return master.getPosition().getValue();
  }

  public double getArmPosRotations() {
    return getPosRotations() / Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
  }

  public void setTargetDegrees(double angleDegrees) {
    targetDegrees = angleDegrees;
  }

  public double determineAngle(Pose2d a, double fkla) {
    return -1;
  }

  public void rotateArmToSpeakerPosition() {
    setTargetDegrees(Constants.Arm.SPEAKER_ANGLE);
  }

  public void rotateArmToRestPosition() {
    setTargetDegrees(0);
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
    //setPosition(targetDegrees);

    SmartDashboard.putString("Command:", this.getCurrentCommand() == null ? "none" : this.getCurrentCommand().getName());
    
    SmartDashboard.putNumber("Absolute Raw", revEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Absolute via Function", getAbsolutePosition());
    SmartDashboard.putNumber("Integrated Rotations: ", getPosRotations());
    SmartDashboard.putNumber("Integrated Current: ", master.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Integrated Error: ", master.getClosedLoopError().getValue());
    SmartDashboard.putNumber("Target Degrees: ", targetDegrees);
    SmartDashboard.putNumber("Arm Rotations: ", getArmPosRotations());
  }
}
