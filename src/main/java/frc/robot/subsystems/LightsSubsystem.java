package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
  private AddressableLED lights;
  private AddressableLEDBuffer ledBuffer;
  private LightSetting currentSetting;
  private int periodicCounter;
  private static LightsSubsystem instance;

  public enum LightSetting {
    FULL_RED {
      @Override
      public int[] apply(int t, int pos) {
        int[] hsv = new int[3];
        hsv[0] = Constants.LED.PURE_RED[0];
        hsv[1] = Constants.LED.PURE_RED[1];
        hsv[2] = Constants.LED.PURE_RED[2];
        return hsv;
      }
    },
    FULL_BLUE {
      @Override
      public int[] apply(int t, int pos) {
        int[] hsv = new int[3];
        hsv[0] = Constants.LED.PURE_BLUE[0];
        hsv[1] = Constants.LED.PURE_BLUE[1];
        hsv[2] = Constants.LED.PURE_BLUE[2];
        return hsv;
      }
    },
    FULL_YELLOW {
      @Override
      public int[] apply(int t, int pos) {
        int[] hsv = new int[3];
        hsv[0] = Constants.LED.PURE_YELLOW[0];
        hsv[1] = Constants.LED.PURE_YELLOW[1];
        hsv[2] = Constants.LED.PURE_YELLOW[2];
        return hsv;
      }
    },
    FULL_RAINBOW {
      @Override
      public int[] apply(int t, int pos) {
        int[] hsv = new int[3];
        final var hue = (((3 * t) % 180) + (pos * 180 / Constants.LED.LED_STRIP_LENGTH)) % 180;
        hsv[0] = hue;
        hsv[1] = 255;
        hsv[2] = 128;
        return hsv;
      }
    };

    private LightSetting() {}

    public abstract int[] apply(int t, int pos);
  }

  public LightsSubsystem() {
    lights = new AddressableLED(Constants.LED.LED_STRIP_PORT);
    ledBuffer = new AddressableLEDBuffer(Constants.LED.LED_STRIP_LENGTH);
    lights.setLength(ledBuffer.getLength());
    lights.start();
    periodicCounter = 0;
    currentSetting = LightSetting.FULL_RAINBOW;
  }

  public static LightsSubsystem getInstance() {
    if (instance == null) {
      instance = new LightsSubsystem();
    }
    return instance;
  }

  public void setLightSetting(LightSetting setting) {
    currentSetting = setting;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("yes is running", periodicCounter);
    if (periodicCounter == 10000) {
      this.setLightSetting(LightSetting.FULL_BLUE);
    }
    for (int i = 0; i < Constants.LED.LED_STRIP_LENGTH; i++) {
      int hsv[] = currentSetting.apply(periodicCounter, i);
      ledBuffer.setHSV(i, hsv[0], hsv[1], hsv[2]);
      // SmartDashboard.putNumber("loc", i);
      // SmartDashboard.putNumber("h", hsv[0]);
      // SmartDashboard.putNumber("s", hsv[1]);
      // SmartDashboard.putNumber("v", hsv[2]);
    }
    lights.setData(ledBuffer);
    periodicCounter++;
  }
}
