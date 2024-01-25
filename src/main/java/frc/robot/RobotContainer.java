// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Pose2d[] noteLocations = {Constants.Notes.Blue.left, Constants.Notes.Blue.middle, Constants.Notes.Blue.right};
  private static SendableChooser<Integer> pickup1choice, pickup2choice;
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

  private void setupChooser() {
    pickup1choice = new SendableChooser<Integer>();
    pickup2choice = new SendableChooser<Integer>();

    pickup1choice.setDefaultOption("do nothing after 1st shoot", 3);
    pickup1choice.addOption("Ring 1 (leftmost robot perspective)", 0); 
    pickup1choice.addOption("Ring 2 (middle)", 1); 
    pickup1choice.addOption("Ring 3 (rightmost robot perspective)", 2); 

    pickup2choice.setDefaultOption("do nothing after 1st pickup", 3);
    pickup2choice.addOption("Ring 1 (leftmost robot perspective)", 0); 
    pickup2choice.addOption("Ring 2 (middle)", 1); 
    pickup2choice.addOption("Ring 3 (right robot perspective)", 2); 

    SmartDashboard.putData(pickup1choice);
    SmartDashboard.putData(pickup2choice);
  }

  private void configureBindings() {
    driveTrain.setDefaultCommand(swerveJoystickCommand);

    // zero-heading
    joystick.circle().onTrue(driveTrain.runOnce(() -> driveTrain.seedFieldRelative(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
    driveTrain.registerTelemetry(logger::telemeterize);

  }
  
  public RobotContainer() {
    // Vibrate joysticks when someone interesting happens!
    // joystick.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1);
    // joystick.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);

    configureBindings();
    setupChooser();
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("THREE NOTE AUTON").andThen(
      new MoveToTarget(driveTrain, noteLocations[pickup1choice.getSelected()])
    ).andThen(
      new MoveToTarget(driveTrain, noteLocations[pickup2choice.getSelected()])
    );
  }
}
