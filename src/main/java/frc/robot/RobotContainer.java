// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandPS4Controller joystick = new CommandPS4Controller(0);
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(
      () -> -joystick.getRawAxis(1),
      () -> -joystick.getRawAxis(0),
      () -> -joystick.getRawAxis(2),
      () -> (joystick.getRawAxis(3) + 1d) / 2d, // joystick L2
      () -> (joystick.getRawAxis(4) + 1d) / 2d, // joystick R2
      driveTrain);

  public final Telemetry logger = new Telemetry();

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry () {
    logger.telemeterize(driveTrain.getState());
  }

  private void configureBindings() {
    driveTrain.setDefaultCommand(swerveJoystickCommand);

    // zero-heading
    joystick.circle().onTrue(driveTrain.runOnce(() -> driveTrain.seedFieldRelative(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
    // joystick.square().whileTrue(new MoveToTarget(driveTrain, new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(Math.PI))));
    // joystick.square().whileTrue(AutoBuilder.followPath(createPath()));
    joystick.square().whileTrue(new MoveToTarget(driveTrain, new Pose2d(new Translation2d(1, 0), new Rotation2d(Math.PI/2.0))));
    driveTrain.registerTelemetry(logger::telemeterize);

  }
  // private PathPlannerPath createPath () {
  //   // Create a list of bezier points from poses. Each pose represents one waypoint.
  //   // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
  //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  //           new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
  //           new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0))
  //   );

  //   // Create the path using the bezier points created above
  //   PathPlannerPath path = new PathPlannerPath(
  //           bezierPoints,
  //           new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
  //           new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  //   );
  //   path.preventFlipping = true;

  //   return path;
  // }
  
  public RobotContainer() {
    // Vibrate joysticks when someone interesting happens!
    // joystick.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1);
    // joystick.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    // autonomous command applies brake
    // final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // return driveTrain.applyRequest(() -> brake);
    return new PathPlannerAuto("New Auto");
  }
}
