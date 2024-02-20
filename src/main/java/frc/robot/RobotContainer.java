// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.subsystems.ArmSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands.AimArmCmd;
import frc.robot.commands.ArmCommands.ArmToNeutralCmd;
import frc.robot.commands.ArmCommands.ArmToPickupCmd;
import frc.robot.commands.Auton.MoveToTarget;
import frc.robot.commands.PeterCommands.RunIntakeUntilDetection;
import frc.robot.commands.PeterCommands.Shoot;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.SwerveCommands.SwerveLockedAngleCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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

  public Command getAutonomousCommand() {

    String autonName = (redAlliance) ? "ThreeNoteAutonRed" : "ThreeNoteAutonBlue";
    SmartDashboard.putString("Auton to be run", autonName);
    SmartDashboard.putBoolean("Red Alliance?", redAlliance);
    return new PathPlannerAuto(autonName)
        .andThen(
            (pickup1choice.getSelected().isEmpty())
                ? new WaitCommand(2.0)
                : MoveToTarget.withMirror(
                    driveTrain, pickup1choice.getSelected().get().getNoteLocation(), redAlliance))
        .andThen(
            (pickup2choice.getSelected().isEmpty())
                ? new WaitCommand(2.0)
                : MoveToTarget.withMirror(
                    driveTrain, pickup2choice.getSelected().get().getNoteLocation(), redAlliance));
  }

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandXboxController joystick =
      new CommandXboxController(Constants.OI.MOVEMENT_JOYSTICK_PORT);
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();

  // TODO: Rename backupCommand (Does not tell us what this does)
  private final Command backupCommand =
      new FunctionalCommand(
          () -> {
            peterSubsystem.resetPreshooterPosition();
          },
          () -> {
            peterSubsystem.reversePreshooterRotations(1);
          },
          (a) -> {
            peterSubsystem.stopPreShooterMotor();
          },
          () -> false);

  // Command runUntilDetection = new RunIntakeUntilDetection(peterSubsystem);
  public final Telemetry logger = new Telemetry();

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    // logger.telemeterize(driveTrain.getState());
  }

  private void configureBindings() {

    Trigger leftShoulderTrigger = joystick.leftBumper();
    Supplier<Double> frontBackFunction =
        () -> ((redAlliance) ? joystick.getLeftY() : -joystick.getLeftY());
    Supplier<Double> leftRightFunction =
        () -> ((redAlliance) ? joystick.getLeftX() : -joystick.getLeftX());
    Supplier<Double> rotationFunction = () -> -joystick.getRightX();
    Supplier<Double> speedFunction = () -> // slowmode when left shoulder is pressed, otherwise fast
        leftShoulderTrigger.getAsBoolean() ? 0d : 1d;

    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);

    // Intake
    joystick
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new ArmToPickupCmd(armSubsystem), new RunIntakeUntilDetection(peterSubsystem)),
                new ParallelCommandGroup(new ArmToNeutralCmd(armSubsystem), backupCommand)));

    // Outtake
    joystick
        .rightTrigger()
        .whileTrue(
            new RunCommand(
                () -> {
                  peterSubsystem.reverseMechanism();
                },
                peterSubsystem));

    Command aimCommand =
        new ParallelCommandGroup(
            new SpinUpShooter(peterSubsystem),
            new AimArmCmd(armSubsystem, driveTrain),
            SwerveLockedAngleCmd.fromPose(
                    frontBackFunction,
                    leftRightFunction,
                    () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                    speedFunction,
                    driveTrain)
                .withToleranceEnd(0.02));

    Command aimCommand2 =
        new ParallelCommandGroup(
            new SpinUpShooter(peterSubsystem),
            new AimArmCmd(armSubsystem, driveTrain),
            SwerveLockedAngleCmd.fromPose(
                    frontBackFunction,
                    leftRightFunction,
                    () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                    speedFunction,
                    driveTrain)
                .withToleranceEnd(0.02) // need this to end so we know we've finished aiming
            );
    // Aim
    joystick.x().whileTrue(aimCommand);

    // Fire
    joystick
        .a()
        .whileTrue(
            new SequentialCommandGroup(
                aimCommand2,
                new ParallelCommandGroup(
                    new Shoot(peterSubsystem),

                    // we need this a second time because the first one ended in the aimCommand,
                    // this time without a tolerance end
                    SwerveLockedAngleCmd.fromPose(
                        frontBackFunction,
                        leftRightFunction,
                        () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                        speedFunction,
                        driveTrain))));

    // speaker snap
    joystick
        .y()
        .whileTrue(
            new SwerveLockedAngleCmd(
                frontBackFunction,
                leftRightFunction,
                () -> new Rotation2d(0),
                speedFunction,
                driveTrain));

    // amp snap
    joystick
        .b()
        .whileTrue(
            new SwerveLockedAngleCmd(
                frontBackFunction,
                leftRightFunction,
                () -> new Rotation2d(-Math.PI / 2d),
                speedFunction,
                driveTrain));

    // When no Commands are being issued, Peter motors should not be moving
    peterSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> {
              peterSubsystem.stopIntake();
              peterSubsystem.stopLeftShooter();
              peterSubsystem.stopRightShooter();
              peterSubsystem.stopPreShooterMotor();
            },
            peterSubsystem));

    // zero-heading
    joystick
        .povDown()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.seedFieldRelative(
                        new Pose2d(new Translation2d(1.25, 5.5), new Rotation2d(0)))));
    driveTrain.registerTelemetry(logger::telemeterize);
  }
}
