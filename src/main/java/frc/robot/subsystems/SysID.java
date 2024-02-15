package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

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

public class SysID extends SubsystemBase {
    private final TalonFX fl = new TalonFX(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    private final TalonFX bl = new TalonFX(Constants.Swerve.BACK_LEFT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    private final TalonFX br = new TalonFX(Constants.Swerve.BACK_RIGHT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    private final TalonFX fr = new TalonFX(Constants.Swerve.FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.Swerve.CANBUS_NAME);
    private TalonFX master;

    private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> master.setControl(m_sysidControl.withOutput(volts.in(Volts))),
                null,
                this));

    public SysID() {
        Follower f = new Follower(Constants.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, false);
        fr.setControl(f);
        fr.setInverted(true);
        br.setControl(f);
        br.setInverted(true);
        bl.setControl(f);

        // fl.getConfigurator().apply(clc);
        // fr.getConfigurator().apply(clc);
        // bl.getConfigurator().apply(clc);
        // br.getConfigurator().apply(clc);

        master = fl;

        setName("Flywheel");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        master.getConfigurator().apply(cfg);

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            master.getPosition(),
            master.getVelocity(),
            master.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        master.optimizeBusUtilization();

        SignalLogger.start();
    }

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(()->master.setControl(m_joystickControl.withOutput(output.getAsDouble())));
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }
}