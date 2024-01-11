// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static class OI {
        public static final int DRIVER_PS4_PORT = 3;
        public static final int ARM_JOYSTICK_PORT = 1;
        public static final int NUMPAD_PORT = 0; 

        // Buttons on PS4 Controller
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

        // Joystick constants 
        public static final double DEADBAND = 0.05;
    }

   public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75; // from swervdrivespecialties.com/products/mk4-swerve-module, model is L2
    public static final double kDriveEncoderTicksPerRotation = 2048;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderTicks2Meter = kDriveEncoderRot2Meter / kDriveEncoderTicksPerRotation;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kPTurning = 0.926;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 0;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 8;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackRightDriveAbsoluteEncoderPort = 9;

        public static final int PIGEON_PORT = 13;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.2242676466703415;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.559135138988495;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.827274329960346;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.2793342163178583;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5106;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxPercentSpeed = 1;
        public static final double kTeleDriveMinPercentSpeed = 0;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
    }

    public static final class ArmConstants {
        public static final double ROTATIONAL_GEAR_RATIO = 1 / 126.15;
        public static final double ROTATIONAL_TICKS2ROT = ROTATIONAL_GEAR_RATIO / 2048;

        public static final double EXTENSION_GEAR_RATIO = 1 / 10.0;
        public static final double EXTENSION_TICKS2ROT = EXTENSION_GEAR_RATIO / 4096.0;

        public static final int POTENTIOMETER_PORT = 3; //confirmed

        public static final int FRICTION_BREAK_PORT = 3; //Changing?
        public static final int INTAKE_MOTOR_PORT = 0; //subject to change

        public static final int BOTTOMHALLEFFECT_PORT = 1; //confirmed
        public static final int TOPHALLEFFECT_PORT = 0; //confirmed
        public static final int ROTATINGMOTOR_PORT = 1; // confirmed
        public static final int EXTENDINGMOTOR_PORT = 14; // confirmed
        
        public static final int RANGE_OF_MOTION = 1800; //confirmed
        public static final int STARTING_POINT = -1191;
        public static final double MAX_ROTATION_SPEED = 0.65; //confirmed
        public static final double MAX_ROTATION_ANGLE_DEG = 125;
        public static final double MAX_EXTENTION_SPEED = 0.5; //confirmed
        public static final double pidPositionToleranceDegrees = 1.0;
        public static final double pidExtentionToleranceInches = 1.0;
        public static final double ANGLE_P = 0.05; //subject to chagne
        public static final double ANGLE_I = 0;
        public static final double ANGLE_D = 0;

        public static final double EXTENTION_P = 0.1; //subject to chagne
        public static final double EXTENTION_I = 0;
        public static final double EXTENTION_D = 0;

        public static final double FRICTION_BREAK_DEADBAND = 0.05;

        // ALL IN DEGREES
        public static final double SHELF_PICKUP_DEG = 78; // IDENTIFY ANGLE
        public static final double SHELF_BACKUP_METERS = Units.inchesToMeters(31 + 9);
        public static final double SHELF_JANK_TICKS = 169381;

        public static final double HIGH_CONE_FRONT_DEG = -75.5;
        public static final double HIGH_CONE_BACK_DEG = -HIGH_CONE_FRONT_DEG;
        public static final double HIGH_CUBE_FRONT_DEG = -86.0;
        public static final double HIGH_CUBE_BACK_DEG = -HIGH_CUBE_FRONT_DEG;

        public static final double MID_CONE_FRONT_DEG = -88.4;
        public static final double MID_CONE_BACK_DEG = -MID_CONE_FRONT_DEG;
        public static final double MID_CUBE_FRONT_DEG = -88.4;
        public static final double MID_CUBE_BACK_DEG = -MID_CUBE_FRONT_DEG;

        public static final double LOW_GOAL_FRONT_DEG = -115.0;
        public static final double LOW_GOAL_BACK_DEG = -LOW_GOAL_FRONT_DEG;

        //ALL IN INCHES
        public static final double HIGH_GOAL_DIST_IN = 52.0;
        public static final double MID_CONE_DIST_IN = 35.0;
        public static final double MID_CUBE_DIST_IN = 38.0;
        public static final double LOW_GOAL_DIST_IN = 38.0;
        public static final double MAX_RETRACTED_DEG = 160;
		

    }

    public static final class ClawConstants {

        public static final double MAX_INTAKE_SPEED = 1.0;
        public static final double MIN_INTAKE_SPEED = 0.0; // subject to change
        public static final double MAX_OUTTAKE_SPEED = -1.0;
        public static final double MIN_OUTTAKE_SPEED = -0.0; // subject to change, must be negative
        public static final int CLAW_SOLENOID_PORT = 2; //Changing

    }

    public static final class AutonConstants {
        public static final double kPDriving = 3.2;
        public static final double kIDriving = 0;
        public static final double kDDriving = 0;

        public static final double kPTurning = 2; //4 og
        public static final double kITurning = 0;
        public static final double kDTurning = 0.69;

        public static final double kVMax = 3;
        public static final double kAMax = 3;
    }

    public static final class DockingConstants {
        public static final double DOCKING_TOLERANCE = 17;
        public static final double LEVEL_TOLERANCE = 12;
        public static final double ENGAGE_TOLERANCE = 9;


        public static final double OSCILLATION_SPEED_INCREMENT = 0.02;
        public static final double DOCKING_SPEED = 2.0;
        public static final double ENGAGE_SPEED = 0.6;
        public static final double kPEngage = 0.08;
        public static final double KIEngage = 0;
        public static final double KDEngage = 0;
    }
}
