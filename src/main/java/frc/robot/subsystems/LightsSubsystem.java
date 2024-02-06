package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class LightsSubsystem implements Subsystem {
    AddressableLED lights;
    AddressableLEDBuffer ledBuffer;
    long periodicCounter;

    private static LightsSubsystem instance;

    public enum LightSetting {
        FULL_RED {
            @Override
            public int[] apply(long t, int pos) {
                int[] hsv = new int[3];
                hsv[0] = Constants.LED.PURE_RED[0];
                hsv[1] = Constants.LED.PURE_RED[1];
                hsv[2] = Constants.LED.PURE_RED[2];
                return hsv;
            }
        },
        FULL_BLUE {
            @Override
            public int[] apply(long t, int pos) {
                int[] hsv = new int[3];
                hsv[0] = Constants.LED.PURE_BLUE[0];
                hsv[1] = Constants.LED.PURE_BLUE[1];
                hsv[2] = Constants.LED.PURE_BLUE[2];
                return hsv;
            }
        },
        FULL_YELLOW {
            @Override
            public int[] apply(long t, int pos) {
                int[] hsv = new int[3];
                hsv[0] = Constants.LED.PURE_YELLOW[0];
                hsv[1] = Constants.LED.PURE_YELLOW[1];
                hsv[2] = Constants.LED.PURE_YELLOW[2];
                return hsv;
            }
        },
        FULL_RAINBOW {
            @Override
            public int[] apply(long t, int pos) {
                int[] hsv = new int[3];
                return hsv;
            }
        };

        private LightSetting() {
            // this.pose = pose;
        }

        // private Pose2d getNoteLocation() {
        // // return this.pose;
        // }
        public abstract int[] apply(long t, int pos);
    }

    public LightsSubsystem() {
        lights = new AddressableLED(Constants.LED.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.LED.LED_STRIP_LENGTH);
        periodicCounter = 0;
    }

    public static LightsSubsystem getInstance() {
        if (instance == null) {
            instance = new LightsSubsystem();
        }
        return instance;
    }


    @Override
    public void periodic() {

    }
}
