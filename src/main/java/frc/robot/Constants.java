// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int PS4_CONTROLLER_PORT_1 = 3;
    public static final int SQUARE_BUTTON_PORT = 1;
    public static final int X_BUTTON_PORT = 2;
    public static final int CIRCLE_BUTTON_PORT = 3;
    public static final int TRIANGLE_BUTTON_PORT = 4;
    public static final int L1_BUTTON_PORT = 5;
    public static final int R1_BUTTON_PORT = 6;
    public static final int L2_BUTTON_PORT = 7;
    public static final int R2_BUTTON_PORT = 8;
    public static final int PS_SHARE_BUTTON_PORT = 9;
    public static final int OPTIONS_BUTTON_PORT = 10;
    public static final int L3_BUTTON_PORT = 11;
    public static final int R3_BUTTON_PORT = 12;
    public static final int PS_BUTTON_PORT = 13;
    public static final int BIG_BUTTON_PORT = 14;
  }

  public static final class Intake {
    public static final int INTAKE_MOTOR_PORT = 14;
    public static final double SHOOTER_SPEED = 0.8;
    public static final int NOTE_DETECTOR_PORT = 7;
    public static final int PRE_SHOOTER_PORT = 3;
    public static final int SHOOTER_PORT_RIGHT = 4;
    public static final int SHOOTER_PORT_LEFT = 5;

    public static final double INTAKE_WHEEL_SPEED_RPM = 100;
    public static final double ROTATIONS_TO_SHOOTER = 5d;
  }

  public static final class FieldDimensions {
    public static final double SPEAKER_HEIGHT_INCHES = 78.0;
    public static final double AMP_HEIGHT_INCHES = 35.0;
    public static final double INTAKE_MODE_HEIGHT_INCHES = 4.0;
    public static final double SPEAKER_HEIGHT_METERS = Units.inchesToMeters(SPEAKER_HEIGHT_INCHES);
    public static final double AMP_HEIGHT_METERS = Units.inchesToMeters(AMP_HEIGHT_INCHES);
    public static final double INTAKE_MODE_HEIGHT_METERS =
        Units.inchesToMeters(INTAKE_MODE_HEIGHT_INCHES);
  }

  public static final class Arm {
    public static final double DEFAULT_ARM_ANGLE = 45.0;
    public final double INTAKE_ANGLE = 0; // subject to change
    public final double AMP_ANGLE = 100; // subject to change
    public final static double ARM_ENCODER_OFFSET = 0; // TODO: Change the offset so that the 0 position is when the arm is at its resting position.
  }
}
