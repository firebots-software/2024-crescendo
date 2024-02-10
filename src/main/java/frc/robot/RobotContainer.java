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
// import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TestCommands.SwerveTest;
import frc.robot.commands.TestCommands.TestArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestEncoderSubsystem;

import java.util.Optional;
import java.util.function.Supplier;

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
  private enum NoteLocation {
    AMPSIDE(Constants.Landmarks.AMPSIDE_NOTE_LOCATION),
    MIDDLE(Constants.Landmarks.MIDDLE_NOTE_LOCATION),
    STAGESIDE(Constants.Landmarks.STAGESIDE_NOTE_LOCATION);

    private final Pose2d pose;

    private NoteLocation(Pose2d pose) {
      this.pose = pose;
    }

    private Pose2d getNoteLocation() {
      return this.pose;
    }
  }

  private static boolean redAlliance;

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  // Options on SmartDashboard that return an integer index that refers to a note location
  private static SendableChooser<Optional<NoteLocation>>
      pickup1choice = new SendableChooser<Optional<NoteLocation>>(),
      pickup2choice = new SendableChooser<Optional<NoteLocation>>();

  private void setupChooser() {
    // // Instantiations
    // pickup1choice = new SendableChooser<Integer>();
    // pickup2choice = new SendableChooser<Integer>();

    pickup1choice.setDefaultOption("SECOND SHOT: DO NOTHING", Optional.empty());
    pickup1choice.addOption("AMPSIDE", Optional.of(NoteLocation.AMPSIDE));
    pickup1choice.addOption("MIDDLE", Optional.of(NoteLocation.MIDDLE));
    pickup1choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));

    pickup2choice.setDefaultOption("THIRD SHOT: DO NOTHING", Optional.empty());
    pickup2choice.addOption("AMPSIDE NOTE", Optional.of(NoteLocation.AMPSIDE));
    pickup2choice.addOption("MIDDLE NOTE", Optional.of(NoteLocation.MIDDLE));
    pickup2choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));

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

  // public Command getAutonomousCommand() {

  //   String autonName = (redAlliance) ? "ThreeNoteAutonRed" : "ThreeNoteAutonBlue";
  //   SmartDashboard.putString("Auton to be run", autonName);
  //   SmartDashboard.putBoolean("Red Alliance?", redAlliance);
  //   return new PathPlannerAuto(autonName)
  //       .andThen(
  //           (pickup1choice.getSelected().isEmpty())
  //               ? new WaitCommand(2.0)
  //               : MoveToTarget.withMirror(
  //                   driveTrain, pickup1choice.getSelected().get().getNoteLocation(), redAlliance))
  //       .andThen(
  //           (pickup2choice.getSelected().isEmpty())
  //               ? new WaitCommand(2.0)
  //               : MoveToTarget.withMirror(
  //                   driveTrain, pickup2choice.getSelected().get().getNoteLocation(), redAlliance));
  // }

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandPS4Controller mjoystick = new CommandPS4Controller(Constants.OI.MOVEMENT_JOYSTICK_PORT);
  private final CommandPS4Controller sjoystick =
      new CommandPS4Controller(Constants.OI.ARM_JOYSTICK_PORT);
  //private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  //private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final TestEncoderSubsystem testEncoderSubsystem = TestEncoderSubsystem.getInstance();
  public final Telemetry logger = new Telemetry();

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    //logger.telemeterize(driveTrain.getState());
  }

  private void configureBindings() {
    // mjoystick.L1();
    // double yes = mjoystick.getRawAxis(0);
    // Supplier
    

  TestArmCommand tac = new TestArmCommand(
    armSubsystem, 
    () -> mjoystick.getRawAxis(0));
    // SwerveJoystickCommand swerveJoystickCommand =
    //     new SwerveJoystickCommand(
    //         () -> ((redAlliance) ? mjoystick.getRawAxis(1) : -mjoystick.getRawAxis(1)),
    //         () -> ((redAlliance) ? mjoystick.getRawAxis(0) : -mjoystick.getRawAxis(0)),
    //         () -> -mjoystick.getRawAxis(2),
    //         () -> (mjoystick.getRawAxis(3) - mjoystick.getRawAxis(4)) / 4d + 0.5,
    //         driveTrain);
    // driveTrain.setDefaultCommand(swerveJoystickCommand);

    // // zero-heading
    // mjoystick
    //     .circle()
    //     .onTrue(
    //         driveTrain.runOnce(
    //             () ->
    //                 driveTrain.seedFieldRelative(
    //                     new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
    // driveTrain.registerTelemetry(logger::telemeterize);

    // This supplier should return only three distinct values: 0.0, -30.0, and 30.0.
    Supplier<Double> swerveAngleOffset = () -> (int)(mjoystick.getRawAxis(0)*1.5) * 30.0;

    SwerveTest swerveTestCommand = new SwerveTest(testEncoderSubsystem, swerveAngleOffset);
    mjoystick.circle().whileTrue(swerveTestCommand);

    //sjoystick.getRawAxis(3); // Trigger
    //sjoystick.getRawAxis(4); // Trigger
  }
}
