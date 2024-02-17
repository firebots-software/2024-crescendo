// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.DebugCommands.IntakeMotorTest;
import frc.robot.commands.DebugCommands.LeftShooterTest;
import frc.robot.commands.DebugCommands.PeterTest;
import frc.robot.commands.DebugCommands.PreShooterTest;
import frc.robot.commands.DebugCommands.RightShooterTest;
import frc.robot.commands.DebugCommands.ShooterTest;
import frc.robot.commands.PeterCommands.RunIntakeUntilDetection;
import frc.robot.commands.PeterCommands.ShootNote;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandPS4Controller mjoystick =
      new CommandPS4Controller(Constants.OI.MOVEMENT_JOYSTICK_PORT);
  private final CommandPS4Controller sjoystick =
      new CommandPS4Controller(Constants.OI.ARM_JOYSTICK_PORT);
  // private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();
  // private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  public final Telemetry logger = new Telemetry();

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  /*  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
  } */

  private final Command backupCommand = new FunctionalCommand(() -> {
    peterSubsystem.resetPreshooterPosition();
  }, () -> {
    peterSubsystem.reversePreshooterRotations(1);
  }, (a) -> {
    peterSubsystem.stopPreShooterMotor();
  }, () -> false);

  Command runUntilDetection = new RunIntakeUntilDetection(peterSubsystem);

  private void configureBindings() {
    var spinUpShooter = new FunctionalCommand(() -> {}, () -> {
      peterSubsystem.spinLeftShooter();
      peterSubsystem.spinRightShooter();
    }, (a) -> {}, () -> {return peterSubsystem.isShooterReady();}, peterSubsystem);

    var shoot = new FunctionalCommand(() -> {}, () -> {
      peterSubsystem.spinLeftShooter();
      peterSubsystem.spinRightShooter();
      peterSubsystem.spinUpPreShooter();
    },  (a) -> {
      peterSubsystem.stopLeftShooter();
      peterSubsystem.stopRightShooter();
      peterSubsystem.stopPreShooterMotor();
    }, () -> {
      return false;
    }, peterSubsystem);

    mjoystick.L2().whileTrue(new SequentialCommandGroup(runUntilDetection, backupCommand));
    mjoystick.R2().whileTrue(new SequentialCommandGroup(spinUpShooter, shoot));

    mjoystick.L1().whileTrue(new RunCommand( () -> {
      peterSubsystem.reverseMechanism();
    }, peterSubsystem));

    peterSubsystem.setDefaultCommand(new RunCommand(() -> {
      peterSubsystem.stopIntake();
      peterSubsystem.stopLeftShooter();
      peterSubsystem.stopRightShooter();
      peterSubsystem.stopPreShooterMotor();
    }, peterSubsystem));
    /*   SwerveJoystickCommand swerveJoystickCommand =
            new SwerveJoystickCommand(
                () -> -mjoystick.getRawAxis(1),
                () -> -mjoystick.getRawAxis(0),
                () -> -mjoystick.getRawAxis(2),
                () -> (mjoystick.getRawAxis(3) - mjoystick.getRawAxis(4) + 2d) / 2d + 0.5,
                driveTrain);
        driveTrain.setDefaultCommand(swerveJoystickCommand);

        // zero-heading
        mjoystick
            .circle()
            .onTrue(
                driveTrain.runOnce(
                    () ->
                        driveTrain.seedFieldRelative(
                            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
        driveTrain.registerTelemetry(logger::telemeterize);

        sjoystick.getRawAxis(3); // Trigger
        sjoystick.getRawAxis(4); // Trigger
    */
    mjoystick.circle().whileTrue(new IntakeMotorTest(peterSubsystem));
    mjoystick.square().whileTrue(new PreShooterTest(peterSubsystem));
    mjoystick.triangle().whileTrue(new ShooterTest(peterSubsystem));
    mjoystick.cross().whileTrue(new LeftShooterTest(peterSubsystem));
    mjoystick.povUp().whileTrue(new RightShooterTest(peterSubsystem));
    mjoystick.povDown().whileTrue(new PeterTest(peterSubsystem));
    mjoystick.povRight().whileTrue(new RunIntakeUntilDetection(peterSubsystem));
    mjoystick.povLeft().whileTrue(new ShootNote(peterSubsystem, peterSubsystem)); 
    /* mjoystick
    .povRight()
    .whileTrue(new PeterTestersSequential(peterSubsystem, peterSubsystem, peterSubsystem)); */
    // peterSubsystem.setDefaultCommand(
    //    new ArmAndPeterCommand(
    //        () -> -mjoystick.getRawAxis(3), () -> -mjoystick.getRawAxis(4), peterSubsystem));
  }

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
