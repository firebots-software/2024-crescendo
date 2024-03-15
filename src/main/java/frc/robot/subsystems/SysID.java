package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class SysID extends SubsystemBase {
  private final TalonFX rt;
  private final TalonFX rb;
  private final TalonFX lt;
  private final TalonFX lb;
  private final TalonFX master;

  private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
  private final VoltageOut m_sysidControl = new VoltageOut(0);
  // private final Follower f = new Follower(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, false);

  private SysIdRoutine m_SysIdRoutine;

  public SysID() {

    rt = new TalonFX(Constants.Arm.RT_PORT, Constants.Arm.CANBUS_NAME);
    rb = new TalonFX(Constants.Arm.RB_PORT, Constants.Arm.CANBUS_NAME);
    lt = new TalonFX(Constants.Arm.LT_PORT, Constants.Arm.CANBUS_NAME);
    lb = new TalonFX(Constants.Arm.LB_PORT, Constants.Arm.CANBUS_NAME);

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(Constants.Arm.LT_PORT, true);
    Follower invertedFollower = new Follower(Constants.Arm.LT_PORT, false);
    rt.setControl(follower);
    rb.setControl(follower);
    lb.setControl(invertedFollower);


    // fl = new TalonFX(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    // bl = new TalonFX(Constants.Swerve.BACK_LEFT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    // br = new TalonFX(Constants.Swerve.BACK_RIGHT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    // fr = new TalonFX(Constants.Swerve.FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);

    // fr.setControl(new Follower(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, true));
    // br.setControl(new Follower(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, true));
    // bl.setControl(new Follower(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, false));
    
    master = lt;

    // fr.setInverted(true);
    // br.setControl(f);
    // br.setInverted(true);
    // bl.setControl(f);
    m_SysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Default ramp rate is acceptable
              Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
              null, // Default timeout is acceptable
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) ->
                  master.setControl(m_sysidControl.withOutput(volts.in(Volts))),
              null,
              this));

    setName("ArmSysID");

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    master.getConfigurator().apply(cfg);

    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, master.getPosition(), master.getVelocity(), master.getMotorVoltage());

    /* Optimize out the other signals, since they're not particularly helpful for us */
    master.optimizeBusUtilization();

    SignalLogger.start();
  }

  public Command joystickDriveCommand(DoubleSupplier output) {
    return run(() -> master.setControl(m_joystickControl.withOutput(output.getAsDouble())));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_SysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_SysIdRoutine.dynamic(direction);
  }
}
