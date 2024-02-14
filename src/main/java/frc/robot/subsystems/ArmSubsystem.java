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

  private boolean initialize = false;
  
  private double targetDegrees;
  private double absEncPretzelClamp = Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL*8/10d;
  private double armHorizontalOffset;

  public ArmSubsystem() {
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

    Slot0Configs s0c = new Slot0Configs().withKP(37).withKI(0).withKD(0);
    armff = new ArmFeedforward(0.1, 0.1, 0.1);
    //r1 = new TalonFX(Constants.Arm.RT_PORT, Constants.Swerve.CANBUS_NAME);
    // r2 = new TalonFX(Constants.Arm.RB_PORT, Constants.Swerve.CANBUS_NAME);
    l1 = new TalonFX(Constants.Arm.LT_PORT, Constants.Swerve.CANBUS_NAME);
    // l2 = new TalonFX(Constants.Arm.LB_PORT, Constants.Swerve.CANBUS_NAME);

    // Follower f = new Follower(Constants.Arm.LT_PORT, false);
    // r1.setControl(f);
    // r1.setInverted(true);
    // r2.setControl(f);
    // r2.setInverted(true);
    // l2.setControl(f);

    

    //r1.getConfigurator().apply(clc);
    // r2.getConfigurator().apply(clc);
    l1.getConfigurator().apply(clc);
    // l2.getConfigurator().apply(clc);

    master = l1;
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
    new Thread(() -> {
      try {
        do {
          Thread.sleep(250);
        } while (!revEncoder.isConnected());
        master.setPosition((getAbsolutePosition())*Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);
        initialize = true;
        armHorizontalOffset = Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET/Constants.Arm.ABSOLUTE_ARM_CONVERSION_FACTOR;
        
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    ).run();
    

    targetDegrees = Constants.Arm.DEFAULT_ARM_ANGLE;
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private void setPosition(double angleDegrees) {
    if(initialize)
    {
      master.setControl(new MotionMagicVoltage(getIntegratedTargetRots(angleDegrees)));
    }
    // .withFeedForward(armff.calculate(getPosRotations() * Math.PI * 2 / 360, 0)));
    // input is in rotations
  }

  private double getIntegratedTargetRots(double angleDegrees){
    double armRots = angleDegrees/360d + armHorizontalOffset;
    return armRots * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
  }

  private double getAbsolutePosition() {
    return(MathUtil.clamp(revEncoder.getAbsolutePosition(), absEncPretzelClamp, 1) - Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL+Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET + 1d) % 1;
  }

  public double getPosRotations() {
    if(!initialize){
      System.out.println("WARNING: Motor Position looked at, but initialization not complete yet. Returning 0");
      return 0;
    }
    return master.getPosition().getValue();
    //return 0;
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
    setTargetDegrees(25);
  }

  public double getArmDegrees()
  {
    return getArmPosRotations() * 360d;
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
    SmartDashboard.putString("Command:", this.getCurrentCommand() == null ? "none" : this.getCurrentCommand().getName());
    //setPosition(targetDegrees);
    SmartDashboard.putNumber("Absolute Raw", revEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Absolute via Function", getAbsolutePosition());
    SmartDashboard.putNumber("Integrated Rotations: ", getPosRotations());
    SmartDashboard.putNumber("Integrated Current: ", master.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Integrated Error: ", master.getClosedLoopError().getValue());
    SmartDashboard.putNumber("Target Degrees: ", targetDegrees);
    SmartDashboard.putNumber("Arm Rotations: ", getArmPosRotations());
    SmartDashboard.putNumber("Arm Degrees", getArmDegrees());
    SmartDashboard.putNumber("Target Integrated Rots: ", getIntegratedTargetRots(targetDegrees));
  }
}
