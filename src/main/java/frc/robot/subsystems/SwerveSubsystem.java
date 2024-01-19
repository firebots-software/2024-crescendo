package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {

    private final AHRS m_otherGyro = new AHRS(SPI.Port.kMXP);
    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.Swerve.kDriveSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs turningCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.Swerve.kTurningSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);

        for (SwerveModule module : Modules) {
            module.getDriveMotor().getConfigurator().apply(driveCurrentLimits);
            module.getSteerMotor().getConfigurator().apply(turningCurrentLimits);
        }
        configurePathPlanner();

    }

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 0, modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
//         public static SwerveModuleState optimize(
//       SwerveModuleState desiredState, Rotation2d currentAngle) {
//     var delta = desiredState.angle.minus(currentAngle);
//     if (Math.abs(delta.getDegrees()) > 90.0) {
//       return new SwerveModuleState(
//           -desiredState.speedMetersPerSecond,
//           desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
//     } else {
//       return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
//     }
//   }
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0), // CHANGE FOR NEW ROBOT
                        new PIDConstants(0.4, 0, 0),
                        Constants.Swerve.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ChassisSpeedsX", getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedsY", getCurrentRobotChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedsRadians", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("gyropitch", m_otherGyro.getPitch());
        SmartDashboard.putNumber("gyroroll", m_otherGyro.getRoll());
        SmartDashboard.putNumber("gyroyaw", m_otherGyro.getYaw());
    }
}
