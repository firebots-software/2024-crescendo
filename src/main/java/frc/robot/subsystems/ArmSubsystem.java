package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ArmSubsystem {

  private TalonFX r1, r2, l1, l2;
  private TalonFX master;
  // private TrapezoidProfile profile;
  // private TrapezoidProfile.Constraints tp;

  private double setPos;

  private MotionMagicConfigs mmc;

  public ArmSubsystem(int portR1, int portR2, int portL1, int portL2) {
    // tp = new TrapezoidProfile.Constraints(10, 20);
    // profile = new TrapezoidProfile(tp);
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

    Slot0Configs s0c =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);

    r1 = new TalonFX(portR1);
    r2 = new TalonFX(portR2);
    l1 = new TalonFX(portL1);
    l2 = new TalonFX(portL2);

    Follower f = new Follower(portR1, false);
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
    setPos = master.getPosition().getValue();

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = 80;
    mmc.MotionMagicAcceleration = 160;
    mmc.MotionMagicJerk = 1600;
    master.getConfigurator().apply(mmc);
  }

  public void setPosition(double angleRotations) {
    MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
    master.setControl(m_request.withPosition(setPos));
  }

  // public void toPosition() {
  //   // Magic Motion:
  //   MotionMagicVoltage m_request = new MotionMagicVoltage(master.getPosition().getValue());
  //   master.setControl(m_request.withPosition(setPos));

  //   // Trapizoidal Motion:
  //   // TrapezoidProfile.State setPoint = new TrapezoidProfile.State(setPos, 0);
  //   // TrapezoidProfile.State currentPoint = new
  //   // TrapezoidProfile.State(master.getPosition().getValue(),master.getVelocity().getValue());

  //   // setPoint = profile.calculate(profile.totalTime(), currentPoint, setPoint);
  //   // PositionDutyCycle m_positionControl = new
  //   // PositionDutyCycle(setPoint.position);
  //   // m_positionControl.Position = setPoint.position;
  //   // m_positionControl.Velocity = setPoint.velocity;
  //   // master.setControl(m_positionControl);
  // }
}
