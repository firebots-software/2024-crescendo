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

  public static final class Peter {
    public static final int INTAKE_MOTOR_PORT = 33;
   // public static final double SHOOTER_SPEED = 3; // Shooter gear ratio: 15:12
    public static final int NOTE_DETECTOR_PORT = 1;
    public static final int PRE_SHOOTER_PORT = 32;
    public static final int SHOOTER_PORT_RIGHT = 30;
    public static final int SHOOTER_PORT_LEFT = 31;

    public static final double INTAKE_WHEEL_SPEED_RPS = 200; // Intake gear ratio: 2:1
    public static final double ROTATIONS_TO_SHOOTER = 300; // Preshooter gear ratio: 4:1
    public static final double SHOOT_WHEEL_SPEED_RPS = 4500.0 / 60.0;
   
    public static final String CANBUS_NAME = "rio";

    public static final double INTAKE_GEAR_RATIO = 2;
    public static final double PRESHOOTER_GEAR_RATIO = 4;
    public static final double SHOOTER_WHEELS_GEAR_RATIOS = 15.0 / 12.0;
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
    public static final double DEFAULT_ARM_ANGLE = 15;
    public final double INTAKE_ANGLE = 0; // subject to change
    public static final double AMP_ANGLE = 100; // subject to change
    public static final double SPEAKER_ANGLE =
        40; // TODO: Replace with the function based on distance
    // public static final double ARM_ENCODER_OFFSET = 0; // TODO: Change the offset so that the 0
    // position is when the arm is at its resting
    // position.
    public static final String CANBUS_NAME = "Patrice the Pineapple";

    public static final int RT_PORT = 14; // Right Top motor
    public static final int RB_PORT = 13; // Right Bottom motor
    public static final int LT_PORT = 12; // Left Top motor
    public static final int LB_PORT = 11; // Left Bottom motor
    public static final int ENCODER_PORT = 0; // subject to change

    public static final double CURRENT_LIMIT = 5.0;
    public static final double S0C_KP = 18.5;
    public static final double ARMFF_KS = 0.1;
    public static final double ARMFF_KG = 0.55;
    public static final double ARMFF_KV = 1.45;
    public static final double MOTIONMAGIC_KV = 1; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_KA = 0.5; // MotionMagic Acceleration in RPS^2 of the arm

    public static final double ABSOLUTE_ARM_CONVERSION_FACTOR = 42d / 18d;
    public static final double INTEGRATED_ABSOLUTE_CONVERSION_FACTOR = 34 + 2.0 / 3.0;
    public static final double INTEGRATED_ARM_CONVERSION_FACTOR =
        ABSOLUTE_ARM_CONVERSION_FACTOR
            * INTEGRATED_ABSOLUTE_CONVERSION_FACTOR; // 80.88888888888888888888888888888888888888;
    public static final double ABSOLUTE_ENCODER_HORIZONTAL = 0.28;
    public static final double ABSOLUTE_HORIZONTAL_OFFSET = 0.05;
  }

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int MOVEMENT_JOYSTICK_PORT = 0;
    public static final int ARM_JOYSTICK_PORT = 1;
  }

  public static class Landmarks {
    // Landmarks on the Blue side can be reflected to show the respective locations on the Blue side
    public static final Pose2d STAGESIDE_NOTE_LOCATION = new Pose2d(2.5, 4.1, new Rotation2d());
    public static final Pose2d MIDDLE_NOTE_LOCATION = new Pose2d(2.5, 5.5, new Rotation2d());
    public static final Pose2d AMPSIDE_NOTE_LOCATION = new Pose2d(2.5, 7, new Rotation2d());
    public static final Pose2d SUBWOOFER_LOCATION_GROUND = new Pose2d(0.6, 5.7, new Rotation2d());
    public static final double SUBWOOFER_TARGET_HEIGHT = 1.93;
    public static final double CENTER_LINE_LOCATION = 8.27;
  }

  public static class Swerve {
    public static class PPConstants {
      public static final PathConstraints PATH_PLANNER_CONSTRAINTS =
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    }

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.8768;
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.7;
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
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    public static final double TURNING_SUPPLY_CURRENT_LIMIT_AMPS = 25.0;
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
