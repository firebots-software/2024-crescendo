package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import java.util.ArrayList;

public class LoggedTalonFX extends TalonFX {

  private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
  private String name;
  private String temperature,
      closedLoopError,
      closedLoopReference,
      position,
      velocity,
      acceleration,
      supplycurrent,
      statorcurrent,
      torquecurrent,
      motorvoltage,
      supplyvoltage;

  public LoggedTalonFX(String deviceName, int deviceId, String canbus) {
    super(deviceId, canbus);
    init();
    name = deviceName;
  }

  public LoggedTalonFX(String deviceName, int deviceId) {
    super(deviceId);
    init();
    name = deviceName;
  }

  public LoggedTalonFX(int deviceId, String canbus) {
    super(deviceId, canbus);
    init();
    name = "motor " + deviceId;
  }

  public LoggedTalonFX(int deviceId) {
    super(deviceId);
    init();
    name = "motor " + deviceId;
  }

  public void init() {
    motors.add(this);
    this.temperature = name + "/temperature(degC)";
    this.closedLoopError = name + "/closedLoopError";
    this.closedLoopReference = name + "/closedLoopReference";
    this.position = name + "/position(rotations)";
    this.velocity = name + "/velocity(rps)";
    this.acceleration = name + "/acceleration(rps2)";
    this.supplycurrent = name + "/current/supply(A)";
    this.statorcurrent = name + "/current/stator(A)";
    this.torquecurrent = name + "/current/torque(A)";
    this.motorvoltage = name + "/voltage/motor(V)";
    this.supplyvoltage = name + "/voltage/supply(V)";
  }

  public static void peroidic() {
    for (LoggedTalonFX l : motors) {
      l.periodic();
    }
  }

  public void periodic() {
    DogLog.log(temperature, this.getDeviceTemp().getValueAsDouble());
    DogLog.log(closedLoopError, this.getClosedLoopError().getValueAsDouble());
    DogLog.log(closedLoopReference, this.getClosedLoopReference().getValueAsDouble());

    DogLog.log(position, this.getPosition().getValueAsDouble());
    DogLog.log(velocity, this.getVelocity().getValueAsDouble());
    DogLog.log(acceleration, this.getAcceleration().getValueAsDouble());

    // Current
    DogLog.log(supplycurrent, this.getSupplyCurrent().getValueAsDouble());
    DogLog.log(statorcurrent, this.getStatorCurrent().getValueAsDouble());
    DogLog.log(torquecurrent, this.getTorqueCurrent().getValueAsDouble());

    // Voltage
    DogLog.log(motorvoltage, this.getMotorVoltage().getValueAsDouble());
    DogLog.log(supplyvoltage, this.getSupplyVoltage().getValueAsDouble());
  }
}
