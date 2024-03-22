package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.AimAtSpeaker;
import frc.robot.commandGroups.BundtShot;
import frc.robot.commandGroups.FireAuton;
import frc.robot.commandGroups.FireTeleop;
import frc.robot.commandGroups.Intake;
import frc.robot.commands.ArmCommands.AlterArmValues;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ArmToAngleCmd.EndBehavior;
import frc.robot.commands.Auton.MoveToTarget;
import frc.robot.commands.Auton.RatchetteDisengage;
import frc.robot.commands.DebugCommands.SmartdashBoardCmd;
// import frc.robot.commands.ArmCommands.AlterArmValues;
import frc.robot.commands.PeterCommands.ShootNoWarmup;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.SwerveCommands.SwerveLockedAngleCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;
import frc.robot.util.NoteLocation;
import frc.robot.util.OtherXBoxController;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final OtherXBoxController joystickA =
      new OtherXBoxController(Constants.OI.JOYSTICK_A_PORT);
  public final OtherXBoxController joystickB =
      new OtherXBoxController(Constants.OI.JOYSTICK_B_PORT);
  // Subsystems
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();
  private final JoystickSubsystem joystickSubsystem = new JoystickSubsystem(joystickA.getHID());
  // Logging
  public final Telemetry logger = new Telemetry();
  // Alliance color
  private Supplier<Boolean> redside = () -> redAlliance;
  private static boolean redAlliance;

  public RobotContainer() {
    // Vibrate joysticks when someone interesting happens!
    // joystick.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);
    configureBindings();
    setupChooser();
  }

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
  }

  private void configureBindings() {
    // Joystick suppliers,
    Trigger leftShoulderTrigger = joystickA.leftBumper();
    Supplier<Double>
        frontBackFunction = () -> ((redAlliance) ? joystickA.getLeftY() : -joystickA.getLeftY()),
        leftRightFunction = () -> ((redAlliance) ? joystickA.getLeftX() : -joystickA.getLeftX()),
        rotationFunction = () -> -joystickA.getRightX(),
        speedFunction =
            () ->
                leftShoulderTrigger.getAsBoolean()
                    ? 0d
                    : 1d; // slowmode when left shoulder is pressed, otherwise fast
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            () -> joystickA.leftTrigger().getAsBoolean(),
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);
    // Intake
    joystickA.rightTrigger().whileTrue(new Intake(peterSubsystem, armSubsystem, joystickSubsystem));
    // Aim
    joystickA
        .x()
        .whileTrue(
            new AimAtSpeaker(
                peterSubsystem,
                armSubsystem,
                driveTrain,
                frontBackFunction,
                leftRightFunction,
                speedFunction,
                redside));
    joystickA
        .a()
        .and(joystickB.rightTrigger(0.5).negate())
        .whileTrue(
            new FireTeleop(
                peterSubsystem,
                armSubsystem,
                driveTrain,
                joystickSubsystem,
                frontBackFunction,
                leftRightFunction,
                speedFunction,
                redside));
    joystickA
        .a()
        .and(joystickB.rightTrigger(0.5))
        .whileTrue(new BundtShot(peterSubsystem, armSubsystem, joystickSubsystem));
    joystickA
        .y()
        .whileTrue(
            new SwerveLockedAngleCmd(
                frontBackFunction,
                leftRightFunction,
                (redAlliance) ? () -> Rotation2d.fromDegrees(180) : () -> Rotation2d.fromDegrees(0),
                speedFunction,
                driveTrain));
    // amp snap
    joystickA
        .b()
        .whileTrue(
            new SwerveLockedAngleCmd(
                frontBackFunction,
                leftRightFunction,
                () -> new Rotation2d(-Math.PI / 2d),
                speedFunction,
                driveTrain));
    joystickA.rightBumper().whileTrue(ArmToAngleCmd.toDuck(armSubsystem));
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
    driveTrain.registerTelemetry(logger::telemeterize);
    // joystick B
    // Outtake
    joystickB
        .leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                    new RunCommand(
                        () -> {
                          peterSubsystem.reverseMechanism();
                        },
                        peterSubsystem),
                    ArmToAngleCmd.toNeutral(armSubsystem).withTolerance(1))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // amp shoot
    joystickB
        .rightBumper()
        .and(joystickB.leftBumper())
        .whileTrue(
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        ArmToAngleCmd.toAmp(armSubsystem).withTolerance(1),
                        MoveToTarget.withMirror(
                                driveTrain,
                                redside,
                                Constants.Landmarks.Amp.POSE.getRotation(),
                                null,
                                MiscUtils.plus(
                                    Constants.Landmarks.Amp.POSE,
                                    new Transform2d(
                                        0d,
                                        -(Units.inchesToMeters(12)
                                            + Constants.Swerve.ROBOT_HALF_WIDTH_METERS),
                                        new Rotation2d())))
                            .andThen(
                                MoveToTarget.withMirror(
                                    driveTrain,
                                    redside,
                                    Constants.Landmarks.Amp.POSE.getRotation(),
                                    null,
                                    MiscUtils.plus(
                                        Constants.Landmarks.Amp.POSE,
                                        new Transform2d(
                                            0d,
                                            -(Units.inchesToMeters(5)
                                                + Constants.Swerve.ROBOT_HALF_WIDTH_METERS),
                                            new Rotation2d())))),
                        new SpinUpShooter(peterSubsystem, true))));
                    // new ShootNoWarmup(peterSubsystem, false),
                    // ArmToAngleCmd.toNeutral(armSubsystem))
                /* .withInterruptBehavior(InterruptionBehavior.kCancelSelf)*/
    // just move arm to amp position
    joystickB
        .a()
        .whileTrue(
            ArmToAngleCmd.toAmp(armSubsystem).withTolerance(1).withReturnToRest(EndBehavior.STAY));
    // shoot
    joystickB
        .y()
        .whileTrue(
            new SequentialCommandGroup(
                new SpinUpShooter(peterSubsystem, true),
                new ShootNoWarmup(peterSubsystem, false).withTimeout(0.5),
                ArmToAngleCmd.toNeutral(armSubsystem)));
    // zero-heading
    joystickB
        .x()
        .onTrue(
            driveTrain
                .runOnce(
                    () ->
                        driveTrain.seedFieldRelative(
                            new Pose2d(
                                new Translation2d(
                                    !redAlliance
                                        ? 1.34 // 1.34
                                        : (Constants.Landmarks.CENTER_LINE_LOCATION * 2 - 1.25),
                                    5.5),
                                Rotation2d.fromDegrees(!redAlliance ? 0 : 180))))
                .andThen(new PrintCommand("pov worked")));
    joystickB.povDown().onTrue(new AlterArmValues(0.25));
    joystickB.povUp().onTrue(new AlterArmValues(-0.25));
  }

  // Constructs a Pose2d array of the note locations by a specific indexing so they can be accessed
  // by the eventual autonomous chooser

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  // Options on SmartDashboard that return an integer index that refers to a note location
  private static SendableChooser<Optional<NoteLocation>>
      pickup1choice = new SendableChooser<Optional<NoteLocation>>(),
      pickup2choice = new SendableChooser<Optional<NoteLocation>>(),
      pickup3choice = new SendableChooser<Optional<NoteLocation>>();
  SendableChooser<String> startchoice = new SendableChooser<String>();

  private void setupChooser() {
    pickup1choice.setDefaultOption("SECOND SHOT: DO NOTHING", Optional.empty());
    pickup1choice.addOption("AMPSIDE", Optional.of(NoteLocation.AMPSIDE));
    pickup1choice.addOption("MIDDLE", Optional.of(NoteLocation.MIDDLE));
    pickup1choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));
    pickup2choice.setDefaultOption("THIRD SHOT: DO NOTHING", Optional.empty());
    pickup2choice.addOption("AMPSIDE NOTE", Optional.of(NoteLocation.AMPSIDE));
    pickup2choice.addOption("MIDDLE NOTE", Optional.of(NoteLocation.MIDDLE));
    pickup2choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));
    pickup3choice.setDefaultOption("FOURTH SHOT: DO NOTHING", Optional.empty());
    pickup3choice.addOption("AMPSIDE", Optional.of(NoteLocation.AMPSIDE));
    pickup3choice.addOption("MIDDLE", Optional.of(NoteLocation.MIDDLE));
    pickup3choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));
    startchoice.setDefaultOption("STARTING POSITION: MIDDLE START", "Mid");
    startchoice.addOption("AMPSIDE START", "Amp");
    startchoice.addOption("STAGESIDE START", "Stage");
    SmartDashboard.putData(pickup1choice);
    SmartDashboard.putData(pickup2choice);
    SmartDashboard.putData(pickup3choice);
    SmartDashboard.putData(startchoice);
  }

  public Command getAutonomousCommand() {
    // NamedCommands.registerCommand("Fire", new FireAuton(peterSubsystem, armSubsystem, driveTrain,
    // 1, redside));
    // NamedCommands.registerCommand("Intake", new Intake(peterSubsystem, armSubsystem,
    // joystickSubsystem));
    // NamedCommands.registerCommand("Ratchette", new RatchetteDisengage(armSubsystem));
    // return new PathPlannerAuto("SamplePath");
    // String autonName = (redAlliance) ? "ThreeNoteAutonRed" : "ThreeNoteAutonBlue";
    // SmartDashboard.putString("Auton to be run", autonName);
    // SmartDashboard.putBoolean("Red Alliance?", redAlliance);
    PathPlannerAuto start =
        new PathPlannerAuto(
            (redAlliance ? "Red" : "Blue")
                .concat(startchoice.getSelected().trim())
                .concat("Start"));
    return new RatchetteDisengage(armSubsystem)
        .andThen(new SmartdashBoardCmd("auton status", "starting"), start)
        // .andThen(new RatchetteDisengage(armSubsystem), new PrintCommand("finished Rachette"))
        .andThen(
            new FireAuton(peterSubsystem, armSubsystem, driveTrain, 1, redside),
            new SmartdashBoardCmd("auton status", "fired 1"))
        .andThen(getAutonShoot(pickup1choice.getSelected(), false))
        .andThen(new SmartdashBoardCmd("auton status", "pickup1 ended"))
        .andThen(getAutonShoot(pickup2choice.getSelected(), false))
        .andThen(new SmartdashBoardCmd("auton status", "pickup2 ended"))
        .andThen(getAutonShoot(pickup3choice.getSelected(), false))
        .andThen(new SmartdashBoardCmd("auton status", "auton finished"));
  }

  public Command getAutonShoot(Optional<NoteLocation> note, boolean backw) {
    return new SmartdashBoardCmd("auton status detail", "BEGIN")
        .andThen(
            (note.isEmpty())
                ? new WaitCommand(2.0)
                : MoveToTarget.withMirror(
                        driveTrain,
                        redside,
                        note.get().getNoteLocation().getRotation(),
                        (backw) ? new Rotation2d(Math.PI) : null,
                        0.2,
                        MiscUtils.plus(
                            note.get().getNoteLocation(),
                            new Translation2d(
                                Units.inchesToMeters(-28d),
                                note.get().getNoteLocation().getRotation())))
                    // note.get()
                    //     .getNoteLocation()
                    //     .plus(new Transform2d(-40d,note.gext().getNoteLocation().getRotation()))))
                    // .plus(new
                    // Transform2d(Units.inchesToMeters(-40)*Math.sin(note.get().getNoteLocation().getRotation().getRadians()), Units.inchesToMeters(-40)*Math.cos(note.get().getNoteLocation().getRotation().getRadians()), new Rotation2d())))
                    .alongWith(
                        new SmartdashBoardCmd("auton intake status", "intake started"),
                        new Intake(peterSubsystem, armSubsystem, joystickSubsystem).withTimeout(3d))
                    .andThen(
                        new FireAuton(peterSubsystem, armSubsystem, driveTrain, 1, redside),
                        new SmartdashBoardCmd("auton status detail", "shot and ended")));
    // .andThen(
    //     new SmartdashBoardCmd("auton status detail", "MTND-DU"),
    //     MoveToTarget.withMirror(
    //             driveTrain,
    //             redside,
    //             null,
    //             0,
    //             note.get().getNoteLocation().getRotation(),
    //             note.get()
    //                 .getNoteLocation()
    //                 .plus(
    //                     new Transform2d(
    //                         Units.inchesToMeters(-18), 0, new Rotation2d())))
    //         .alongWith(
    //             new SmartdashBoardCmd("auton intake status", "intake started"),
    //             new Intake(peterSubsystem, armSubsystem, joystickSubsystem)
    //                 .withTimeout(2.75d)))
    // // .andThen(
    // //     MoveToTarget.withMirror(
    // //         driveTrain,
    // //         redside,
    // //         NoteLocation.MIDDLE
    // //             .getNoteLocation()
    // //             .plus(new Transform2d(Units.inchesToMeters(-45), 0, new
    // Rotation2d()))))
    // .andThen(
    //     new FireAuton(peterSubsystem, armSubsystem, driveTrain, 1, redside),
    //     new SmartdashBoardCmd("auton status detail", "shot and ended")));
  }
}
