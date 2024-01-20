// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.PhotonVision;
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
  private final CommandPS4Controller joystick = new CommandPS4Controller(3);
  private final SwerveSubsystem drivetrain = Constants.Swerve.DriveTrain;
  private final PhotonVision photonVision = PhotonVision.getInstance();
  private final SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(
      () -> joystick.getRawAxis(1),
      () -> joystick.getRawAxis(0),
      () -> joystick.getRawAxis(2),
      () -> (joystick.getRawAxis(3) + 1d) / 2d, // joystick L2
      () -> (joystick.getRawAxis(4) + 1d) / 2d, // joystick R2
      drivetrain);

  public final Telemetry logger = new Telemetry();

  // private Command runAuto = drivetrain.getAutoPath("Tests");

  public void doTelemetry () {
    logger.telemeterize(drivetrain.getState());
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(swerveJoystickCommand);

    // zero-heading
    joystick.circle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
    drivetrain.registerTelemetry(logger::telemeterize);

    // Align to Tag
    int tagID = 4;
    joystick.triangle().whileTrue(new AlignToTag(photonVision, drivetrain, tagID));
  }

  public RobotContainer() {
    // Vibrate joysticks when someone interesting happens!
    // joystick.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1);
    // joystick.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);

    configureBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auton");
    // return runAuto;
  }
}