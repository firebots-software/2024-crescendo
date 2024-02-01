package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class LEDSubsystem implements Subsystem {

  private static LEDSubsystem instance;
  private static int counter;
  private static int lightsCount;
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  public LEDSubsystem(){
        counter = 0;
        lightsCount = Constants.LIGHTS.LIGHT_SETTINGS_COUNT;
        m_led = new AddressableLED(Constants.LIGHTS.LED_PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS.LED_BUFFER_LENGTH);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
  } 

  public void incrementer() {
    if (counter == lightsCount-1) {
        counter = 0;
    } else
        counter++;
  }

  public void lightSetting0() {
    // ALL YELLOW - SIGNALS FOR CONE
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i,
                Constants.LIGHTS.YELLOW_3501[0],
                Constants.LIGHTS.YELLOW_3501[1],
                Constants.LIGHTS.YELLOW_3501[2]);
    }
    m_led.setData(m_ledBuffer);
  }

  public void lightSetting1() {
    // ALL BLUE - SIGNALS FOR CUBE
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i,
                Constants.LIGHTS.BLUE_3501[0],
                Constants.LIGHTS.BLUE_3501[1],
                Constants.LIGHTS.BLUE_3501[2]);
    }
    m_led.setData(m_ledBuffer);
  }

  public void lightSetting2(int firstRed) {
    // GRADIENT RED-YELLOW
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, 255, Math.abs(i-firstRed)*3, 0);
    }
  }

  public static LEDSubsystem getInstance() {
      if (instance == null) {
        instance =
            new LEDSubsystem();
      }
      return instance;
    }
}

