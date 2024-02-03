// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.PhotonVision;
import java.io.IOException;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
  // The robot's subsystems and commands are defined here...
  private final PhotonVision m_exampleSubsystem;

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @throws IOException
   */
  public RobotContainer() {
    // Configure the trigger bindings
    m_exampleSubsystem = PhotonVision.getInstance();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // public Command getAutonomousCommand() {
  //   // autonomous command applies brake
  //   // final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //   // return driveTrain.applyRequest(() -> brake);

  //   //return new PathPlannerAuto("New Auto");
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
