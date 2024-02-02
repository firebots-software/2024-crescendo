package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class LEDSubsystem implements Subsystem {

  private static LEDSubsystem instance;
  // private static int counter;
  // private static int lightsCount;
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  public LEDSubsystem() {
    // counter = 0;
    m_led = new AddressableLED(Constants.LIGHTS.LED_PWM_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS.LED_BUFFER_LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void rainbow() {
    // For every pixel
    int m_rainbowFirstPixelHue = 0;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void basicTest() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, 0, 0, 0);
    }
  }

  // public void lightSetting0() {
  //   // ALL YELLOW - SIGNALS FOR CONE
  //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //     m_ledBuffer.setRGB(
  //         i,
  //         Constants.LIGHTS.YELLOW_3501[0],
  //         Constants.LIGHTS.YELLOW_3501[1],
  //         Constants.LIGHTS.YELLOW_3501[2]);
  //   }
  //   m_led.setData(m_ledBuffer);
  // }

  // public void lightSetting1() {
  //   // ALL BLUE - SIGNALS FOR CUBE
  //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //     m_ledBuffer.setRGB(
  //         i,
  //         Constants.LIGHTS.BLUE_3501[0],
  //         Constants.LIGHTS.BLUE_3501[1],
  //         Constants.LIGHTS.BLUE_3501[2]);
  //   }
  //   m_led.setData(m_ledBuffer);
  // }

  // public void lightSetting2(int firstRed) {
  //   // GRADIENT RED-YELLOW
  //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //     m_ledBuffer.setHSV(i, 255, Math.abs(i - firstRed) * 3, 0);
  //   }
  // }

  public void intaking() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 204); //yellow
    }
    m_led.setData(m_ledBuffer);
  }

  public void noteDetected() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 255, 0); //green
    }
    m_led.setData(m_ledBuffer);
  }

  public void shootingInProgress() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 204); //yellow
    }
    m_led.setData(m_ledBuffer);
  }

  public void noteShot() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 255, 0); //green
    }
    m_led.setData(m_ledBuffer);
  }



  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }
}
