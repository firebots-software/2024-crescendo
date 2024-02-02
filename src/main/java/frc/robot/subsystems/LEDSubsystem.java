package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class LEDSubsystem implements Subsystem {
  private static LEDSubsystem instance;
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  public LEDSubsystem() {
    m_led = new AddressableLED(Constants.LIGHTS.LED_PWM_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS.LED_BUFFER_LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  // @Override
  // public void periodic() {

  // }
}
