package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
  // instance of SwerveSubsystem
  private static SwerveSubsystem instance;

  // Constructor allows for custom odometry update frequency
  public SwerveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {

    // Sets the drivetrain constants, odometry update frequency and constants for
    // the swerve modules
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    // Sets the supply current limits for the swerve modules (for driving and
    // turning)
    CurrentLimitsConfigs driveCurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Swerve.DRIVE_STATOR_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Swerve.DRIVE_SUPPLY_CURRENT_LIMIT_AMPS);

    CurrentLimitsConfigs steerCurrentLimits =
        new CurrentLimitsConfigs()
            // .withStatorCurrentLimit(Constants.Swerve.STEER_STATOR_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimit(Constants.Swerve.TURNING_SUPPLY_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true);

    for (SwerveModule module : Modules) {
      module.getDriveMotor().getConfigurator().apply(driveCurrentLimits);
      module.getSteerMotor().getConfigurator().apply(steerCurrentLimits);
    }

    // Configures the holonomic auto builder
    configurePathPlanner();
  }

  // Constructor for default odometry update frequency
  public SwerveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
  }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance =
          new SwerveSubsystem(
              Constants.Swerve.DRIVETRAIN_CONSTANTS,
              Constants.Swerve.FRONT_LEFT,
              Constants.Swerve.FRONT_RIGHT,
              Constants.Swerve.BACK_LEFT,
              Constants.Swerve.BACK_RIGHT);
    }
    return instance;
  }

  // Applies swerve request to drivetrain
  public Command getSwerveRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private void configurePathPlanner() {
    // Sets radius of drive base (using module locations)
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    // Holonomic auto builder
    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                autoRequest
                    .withSpeeds(speeds)
                    .withDriveRequestType(
                        DriveRequestType.Velocity)), // Consumer of ChassisSpeeds to drive the
        // robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(3, 0, 0), // CHANGE FOR NEW ROBOT
            new PIDConstants(3, 0, 0),
            Constants.Swerve.SPEED_AT_12V_METERS_PER_SECOND,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> false, // Change this if the path needs to be flipped on red vs blue
        this); // Subsystem for requirements
  }

  /**
   * @param pathName Name of Path (in PathPlanner)
   * @return The name of the path
   */
  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /**
   * @return Robot's current Chassis Speeds
   */
  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  @Override
  public void periodic() {
    DogLog.log("chassis/speed_x", getCurrentRobotChassisSpeeds().vxMetersPerSecond);
    DogLog.log("chassis/speed_y", getCurrentRobotChassisSpeeds().vyMetersPerSecond);
    DogLog.log("chassis/rotation_speed_radps", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
    // Chassis Speeds information
    // SmartDashboard.putNumber("ChassisSpeedsX", getCurrentRobotChassisSpeeds().vxMetersPerSecond);
    // SmartDashboard.putNumber("ChassisSpeedsY", getCurrentRobotChassisSpeeds().vyMetersPerSecond);
    // SmartDashboard.putNumber(
    //    "ChassisSpeedsRadians", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
  }
}
