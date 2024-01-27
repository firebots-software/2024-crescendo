// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.MoveToTarget;
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

  // Constructs a Pose2d array of the note locations by a specific indexing so they can be accessed
  // by the eventual autonomous chooser
  private final Pose2d[] noteLocations = {
    Constants.Landmarks.LEFT_NOTE_LOCATION,
    Constants.Landmarks.MIDDLE_NOTE_LOCATION,
    Constants.Landmarks.RIGHT_NOTE_LOCATION,
    null
  };

  // Options on SmartDashboard that return an integer index that refers to a note location
  private static SendableChooser<Integer> pickup1choice = new SendableChooser<Integer>(),
      pickup2choice = new SendableChooser<Integer>();

  private void setupChooser() {
    // // Instantiations
    // pickup1choice = new SendableChooser<Integer>();
    // pickup2choice = new SendableChooser<Integer>();

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

  public RobotContainer() {
    // Vibrate joysticks when someone interesting happens!
    // joystick.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1);
    // joystick.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);

    configureBindings();
    setupChooser();
  }

  public Command getAutonomousCommand() {
    boolean redAlliance = false;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      redAlliance = true;
    }
    return new PathPlannerAuto("ThreeNoteAuton")
        .andThen(
            (noteLocations[pickup1choice.getSelected()] == null)
                ? new WaitCommand(2.0)
                : MoveToTarget.withMirror(
                    driveTrain, noteLocations[pickup1choice.getSelected()], redAlliance))
        .andThen(
            (noteLocations[pickup2choice.getSelected()] == null)
                ? new WaitCommand(2.0)
                : MoveToTarget.withMirror(
                    driveTrain, noteLocations[pickup2choice.getSelected()], redAlliance));
  }

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandPS4Controller joystick =
      new CommandPS4Controller(Constants.OI.JOYSTICK_PORT);
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  public final Telemetry logger = new Telemetry();

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
  }

  private void configureBindings() {
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            () -> -joystick.getRawAxis(1),
            () -> -joystick.getRawAxis(0),
            () -> -joystick.getRawAxis(2),
            () -> (joystick.getRawAxis(3) - joystick.getRawAxis(4) + 2d) / 2d + 0.5,
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);

    // zero-heading
    joystick
        .circle()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.seedFieldRelative(
                        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
    driveTrain.registerTelemetry(logger::telemeterize);
  }
}
