// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandPS4Controller joystick = new CommandPS4Controller(3);
  private final SwerveSubsystem drivetrain = Constants.DriveTrain;
  private final SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(
        () -> joystick.getRawAxis(1),
        () -> joystick.getRawAxis(0),
        () -> joystick.getRawAxis(2),
        () -> (joystick.getRawAxis(3) + 1d) / 2d, // joystick L2
        () -> (joystick.getRawAxis(4) + 1d) / 2d, // joystick R2
        drivetrain
        );

    private void configureBindings() {
    drivetrain.setDefaultCommand(swerveJoystickCommand);
    

    // zero-heading
    joystick.R1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
