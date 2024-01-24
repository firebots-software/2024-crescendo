package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_PORT = 0;
  }

  public static class Swerve {

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.8768;
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.7;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.5;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE = (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
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
        new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

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

    private static final double DRIVE_GEAR_RATIO =
        6.746031746031747; // 6.12 for new robot: CHANGE FOR NEW ROBOT
    private static final double STEEP_GEAR_RATIO = 21.428571428571427;
    private static final double WHEEL_RADIUS_INCHES = 2;

    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final String CANBUS_NAME = "";
    private static final int PIGEON_ID = 13;

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
            .withSteerMotorGearRatio(STEEP_GEAR_RATIO)
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

    // CHANGE FOR NEW ROBOT:
    // Front Right
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 7;
    private static final int FRONT_LEFT_ENCODER_ID = 11;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.091796875;

    private static final double FRONT_LEFT_X_POS_INCHES = 12.25;
    private static final double FRONT_LEFT_Y_POS_INCHES = 12.25;

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 2;
    private static final int FRONT_RIGHT_ENCODER_ID = 10;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.1298828125;

    private static final double FRONT_RIGHT_X_POS_INCHES = 12.25;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -12.25;

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 6;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 5;
    private static final int BACK_LEFT_ENCODER_ID = 12;
    private static final double BACK_LEFT_ENCODER_OFFSET = -0.36181640625;

    private static final double BACK_LEFT_X_POS_INCHES = -12.25;
    private static final double BACK_LEFT_Y_POS_INCHES = 12.25;

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 4;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 8;
    private static final int BACK_RIGHT_ENCODER_ID = 9;
    private static final double BACK_RIGHT_ENCODER_OFFSET = -0.03857421875;

    private static final double BACK_RIGHT_X_POS_INCHES = -12.25;
    private static final double BACK_RIGHT_Y_POS_INCHES = -12.25;

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
