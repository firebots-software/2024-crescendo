// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
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

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5106;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxPercentSpeed = 1;
        public static final double kTeleDriveMinPercentSpeed = 0;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }
}
