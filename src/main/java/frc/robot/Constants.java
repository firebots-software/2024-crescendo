package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  public static class LED {
    public static final int LED_STRIP_LENGTH = 34;
    public static final int LED_STRIP_PORT = 7;
    public static final int[] PURE_RED = {0, 100, 100};
    public static final int[] PURE_BLUE = {201, 100, 100};
    public static final int[] PURE_YELLOW = {61, 100, 100};
  }

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

  public static final class MotorConstants {
    public final int PORT;
    public final boolean REVERSED;
    public final double GEAR_RATIO;
    public final double STATOR_CURRENT_LIMIT_AMPS;
    public final double SPEED_RPS;
    public final double SPEED_VOLTAGE;

    private MotorConstants(
        int port,
        boolean reversed,
        double gearRatio,
        double statorCurrent,
        double speed,
        double voltage) {
      PORT = port;
      REVERSED = reversed;
      GEAR_RATIO = gearRatio;
      STATOR_CURRENT_LIMIT_AMPS = statorCurrent;
      SPEED_RPS = speed;
      SPEED_VOLTAGE = voltage;
    }

    public static MotorConstants speedControled(
        int port, boolean reversed, double gearRatio, double statorCurrent, double speed) {
      return new MotorConstants(port, reversed, gearRatio, statorCurrent, speed, 0);
    }

    public static MotorConstants voltageControlled(
        int port, boolean reversed, double gearRatio, double statorCurrent, double voltage) {
      return new MotorConstants(port, reversed, gearRatio, statorCurrent, 0, voltage);
    }
  }

  public static final class Pooer {
    public static final int NOTE_DETECTOR_PORT = 1;
    public static final String CANBUS_NAME = "rio";

    public static final ShooterType SHOOTER = ShooterType.PIPER;

    public static enum ShooterType {
      PETER(
          MotorConstants.speedControled(30, true, 12d / 15d, 40.0, 4500d / 60d),
          MotorConstants.speedControled(31, false, 12d / 15d, 40.0, 4500d / 60d),
          MotorConstants.voltageControlled(32, false, 4d / 1d, 25.0, 9d),
          MotorConstants.speedControled(33, true, 2d / 1d, 50.0, 200d)),
      PIPER(
          MotorConstants.speedControled(35, false, 24d / 18d, 40.0, 3000d / 60d),
          MotorConstants.speedControled(34, false, 24d / 18d, 40.0, 3000d / 60d),
          MotorConstants.voltageControlled(32, true, 4d / 1d, 25.0, 9d),
          MotorConstants.speedControled(33, true, 2d / 1d, 50.0, 200d));
      public final MotorConstants SHOOTER_1, SHOOTER_2, PRESHOOTER, INTAKE;

      ShooterType(
          MotorConstants shooter1,
          MotorConstants shooter2,
          MotorConstants preshooter,
          MotorConstants intake) {
        SHOOTER_1 = shooter1;
        SHOOTER_2 = shooter2;
        PRESHOOTER = preshooter;
        INTAKE = intake;
      }
    }
  }

  public static final class Arm {
    public static final double ARM_STATOR_CURRENT_LIMIT_AMPS = 40.0;
    public static final double DEFAULT_ARM_ANGLE = 56.12;
    public static final double INTAKE_ANGLE = 4; // subject to change
    public static final double AMP_ANGLE = 90; // subject to change
    // public static final double ARM_ENCODER_OFFSET = 0; // TODO: Change the offset so that the 0
    // position is when the arm is at its resting
    // position.
    public static final String CANBUS_NAME = "Patrice the Pineapple";

    public static final int RT_PORT = 14; // Right Top motor
    public static final int RB_PORT = 13; // Right Bottom motor
    public static final int LT_PORT = 12; // Left Top motor
    public static final int LB_PORT = 11; // Left Bottom motor
    public static final int ENCODER_PORT = 0; // subject to change

    public static final double CURRENT_LIMIT = 8.0;
    public static final double S0C_KP = 1.2;
    public static final double ARMFF_KS = 0.15;
    public static final double ARMFF_KG = 0.2;
    public static final double ARMFF_KV = 2.49;
    public static final double MOTIONMAGIC_KV = 1; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_KA = 2.2; // MotionMagic Acceleration in RPS^2 of the arm

    public static final double FEET_TO_METERS_CONVERSION_FACTOR = 0.3048;
    public static final double ABSOLUTE_ARM_CONVERSION_FACTOR = 42d / 18d;
    public static final double INTEGRATED_ABSOLUTE_CONVERSION_FACTOR = 55.9867;
    public static final double INTEGRATED_ARM_CONVERSION_FACTOR =
        ABSOLUTE_ARM_CONVERSION_FACTOR
            * INTEGRATED_ABSOLUTE_CONVERSION_FACTOR; // 130.63563333333335;
    public static final double ABSOLUTE_ENCODER_HORIZONTAL = 0.6547;
    public static final double ABSOLUTE_HORIZONTAL_OFFSET = 0.05;
    public static double ARM_INTERMAP_OFFSET = 4;
    public static final InterpolatingDoubleTreeMap INTERMAP = new InterpolatingDoubleTreeMap();

    static {
      UPDATE_INTERMAP();
      // INTERMAP.put(
      //     1.34,
      //     6d + ARM_INTERMAP_OFFSET); // measurements of distance are from front of robot bumper
      // to
      // // wall
      // INTERMAP.put(2.1, 17d + ARM_INTERMAP_OFFSET);
      // INTERMAP.put(Units.feetToMeters(9) + Units.inchesToMeters(17), 23.5d +
      // ARM_INTERMAP_OFFSET);
    }

    public static void UPDATE_INTERMAP() {
      INTERMAP.clear();
      INTERMAP.put(
          1.34,
          6d + ARM_INTERMAP_OFFSET); // measurements of distance are from front of robot bumper to
      // wall
      INTERMAP.put(2.1, 17d + ARM_INTERMAP_OFFSET);
      INTERMAP.put(Units.feetToMeters(9) + Units.inchesToMeters(17), 23.5d + ARM_INTERMAP_OFFSET);
    }

    // public static final InterpolatingDoubleTreeMap INTERMAP2 = new InterpolatingDoubleTreeMap();
    // static {
    //   INTERMAP2.put(1.34, 6d + 5); // measurements of distance are from front of robot bumper to
    // wall
    //   INTERMAP2.put(2.1, 17d + 5);
    //   INTERMAP2.put(Units.feetToMeters(9) + Units.inchesToMeters(17), 23.5d + 5);
    // }
  }

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;
    public static final int JOYSTICK_B_PORT = 1;

    public enum XBoxButtonID {
      /** A. */
      A(1),
      /** B. */
      B(2),
      /** X. */
      X(3),
      /** Y. */
      Y(4),
      /** Left bumper. */
      LeftBumper(5),
      /** Right bumper. */
      RightBumper(6),
      /** Left stick. */
      LeftStick(9),
      /** Right stick. */
      RightStick(10),
      /** Back. */
      Back(7),
      /** Start. */
      Start(8);
      public final int value;

      XBoxButtonID(int value) {
        this.value = value;
      }
    }

    public enum AxisID {
      /** Left X. */
      LeftX(0),
      /** Right X. */
      RightX(4),
      /** Left Y. */
      LeftY(1),
      /** Right Y. */
      RightY(5),
      /** Left trigger. */
      LeftTrigger(2),
      /** Right trigger. */
      RightTrigger(3);

      /** Axis value. */
      public final int value;

      AxisID(int value) {
        this.value = value;
      }
    }
  }

  public static class Landmarks {
    // Landmarks on the Blue side can be reflected to show the respective locations on the Blue side
    public static final Pose2d STAGESIDE_NOTE_LOCATION =
        new Pose2d(2.8956, 4.0522, new Rotation2d());
    public static final Pose2d MIDDLE_NOTE_LOCATION = new Pose2d(2.8956, 5.5, new Rotation2d());
    public static final Pose2d AMPSIDE_NOTE_LOCATION = new Pose2d(2.8956, 6.9478, new Rotation2d());
    public static final Pose2d SUBWOOFER_LOCATION = new Pose2d(0.6, 5.7, new Rotation2d());
    public static final double CENTER_LINE_LOCATION = 8.27;

    public static final class Speaker {
      public static final double HEIGHT_INCHES = 78.0;
      public static final double HEIGHT_METERS = Units.inchesToMeters(HEIGHT_INCHES);
      public static final Pose2d POSE = new Pose2d(new Translation2d(0, 5.5), new Rotation2d(0));
    }

    public static final class Amp {
      public static final double AMP_HEIGHT_INCHES = 35.0;
      public static final double AMP_HEIGHT_METERS = Units.inchesToMeters(AMP_HEIGHT_INCHES);
      public static final Pose2d POSE =
          new Pose2d(new Translation2d(1.81, 8.11), new Rotation2d(-Math.PI / 2)); // isnt right
      // new Pose2d(new Translation2d(1.84 ,8.2), new Rotation2D(-Math.PI/2));
    }

    public static final double INTAKE_MODE_HEIGHT_INCHES = 4.0;
    public static final double INTAKE_MODE_HEIGHT_METERS =
        Units.inchesToMeters(INTAKE_MODE_HEIGHT_INCHES);
  }

  public static class Swerve {
    public static final Pose2d ROBOT_HALF_WIDTH =
        new Pose2d(Units.inchesToMeters(24), 0, new Rotation2d());
    public static final double ROBOT_HALF_WIDTH_METERS = 0.408;

    public static class PPConstants {
      public static final PathConstraints PATH_PLANNER_CONSTRAINTS =
          new PathConstraints(
              3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // TODO: Increase the auton velocity
    }

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.8768;
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 1.0;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS =
        new Slot0Configs().withKP(50).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(0.18014)
            .withKI(0)
            .withKD(0)
            .withKS(-0.023265)
            .withKV(0.12681)
            .withKA(0.058864);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double SLIP_CURRENT_AMPS = 100.0;
    public static final double DRIVE_STATOR_CURRENT_LIMIT_AMPS = 90.0;
    public static final double STEER_STATOR_CURRENT_LIMIT_AMPS = 40.0;

    public static final double DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double TURNING_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12V_METERS_PER_SECOND = 4.73;

    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO = 6.12; // 6.12 for new robot: CHANGE FOR NEW ROBOT
    private static final double STEER_GEAR_RATIO = 21.428571428571427;
    private static final double WHEEL_RADIUS_INCHES = 2;

    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final String CANBUS_NAME = "Patrice the Pineapple";
    private static final int PIGEON_ID = 40;

    // These are only used for simulation
    // private static final double kSteerInertia = 0.00001;
    // private static final double kDriveInertia = 0.001;
    // // Simulated voltage necessary to overcome friction
    // private static final double kSteerFrictionVoltage = 0.25;
    // private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
        new SwerveDrivetrainConstants().withPigeon2Id(PIGEON_ID).withCANbusName(CANBUS_NAME);

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS_INCHES)
            .withSlipCurrent(SLIP_CURRENT_AMPS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSpeedAt12VoltsMps(SPEED_AT_12V_METERS_PER_SECOND)
            // .withSteerInertia(kSteerInertia) // used in simulation
            // .withDriveInertia(kDriveInertia) // used in simulation
            // .withSteerFrictionVoltage(kSteerFrictionVoltage) // used in simulation
            // .withDriveFrictionVoltage(kDriveFrictionVoltage) // used in simulation
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(STEER_MOTOR_REVERSED);

    private static final double moveCOMY = 0.046007;
    private static final double moveCOMX = 3.36044;

    // Front Left
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
    private static final int FRONT_LEFT_ENCODER_ID = 21;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.472412109375;

    private static final double FRONT_LEFT_X_POS_INCHES = 11.26 - moveCOMX;
    private static final double FRONT_LEFT_Y_POS_INCHES = 11.417 - moveCOMY;

    // Front Right
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    private static final int FRONT_RIGHT_ENCODER_ID = 22;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.436767578125;

    private static final double FRONT_RIGHT_X_POS_INCHES = 11.26 - moveCOMX;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -11.417 - moveCOMY;

    // Back Left
    private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_ENCODER_ID = 20;
    private static final double BACK_LEFT_ENCODER_OFFSET = -0.165283203125;

    private static final double BACK_LEFT_X_POS_INCHES = -11.26 - moveCOMX;
    private static final double BACK_LEFT_Y_POS_INCHES = 11.417 - moveCOMY;

    // Back Right
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    private static final int BACK_RIGHT_ENCODER_ID = 23;
    private static final double BACK_RIGHT_ENCODER_OFFSET = -0.336181640625;

    private static final double BACK_RIGHT_X_POS_INCHES = -11.26 - moveCOMX;
    private static final double BACK_RIGHT_Y_POS_INCHES = -11.417 - moveCOMY;

    public static final SwerveModuleConstants FRONT_LEFT =
        ConstantCreator.createModuleConstants(
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES),
            Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants FRONT_RIGHT =
        ConstantCreator.createModuleConstants(
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES),
            Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE);
    public static final SwerveModuleConstants BACK_LEFT =
        ConstantCreator.createModuleConstants(
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_ENCODER_ID,
            BACK_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_LEFT_X_POS_INCHES),
            Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants BACK_RIGHT =
        ConstantCreator.createModuleConstants(
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES),
            Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE);
  }
}
