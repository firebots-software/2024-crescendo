package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.BundtShot;
import frc.robot.commandGroups.FireTeleop;
import frc.robot.commandGroups.Intake;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.OtherXBoxController;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final OtherXBoxController joystick =
      new OtherXBoxController(Constants.OI.JOYSTICK_A_PORT);
  private final OtherXBoxController joystickSpeedController = new OtherXBoxController(2);
  // Subsystems
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();
  private final JoystickSubsystem joystickSubsystem = new JoystickSubsystem(joystick.getHID());
  // Logging
  public final Telemetry logger = new Telemetry();
  
  public RobotContainer() {
    configureBindings();
  }

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
  }

  public boolean isRedAlliance() {
    boolean redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);

    return redAlliance;
  }

  private void configureBindings() {
    // For outreach, we only want
    // 1. drive
    // 2. intake
    // 3. fire at set angle

    // Driving
    //     * Left joystick = translation
    //     * Right joystick = rotation
    //     * Right shoulder = speed increase
    // Speed is determined by 
    // Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT for when right shoulder is not pressed
    // Constants.Swerve.TELE_DRIVE_FAST_MODE_SPEED_PERCENT for when right shoulder is pressed
    Trigger rightShoulderTrigger = joystickSpeedController.rightBumper();
    Supplier<Double>
        frontBackFunction = () -> -joystick.getLeftY(),
        leftRightFunction = () -> -joystick.getLeftX(),
        rotationFunction = () -> -joystick.getRightX(),
        speedFunction = () -> rightShoulderTrigger.getAsBoolean() ? 1d : 0d;

    Supplier<Boolean> redAllianceSupplier = () -> isRedAlliance();
    
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction,
            () -> false, // we never want robot relative
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);
    driveTrain.registerTelemetry(logger::telemeterize);

    // Intake - left trigger
    // Angle controlled by Constants.Arm.INTAKE_ANGLE
    joystick.leftTrigger().whileTrue(new Intake(peterSubsystem, armSubsystem, joystickSubsystem));
    
    // Shoot - right trigger
    // Angle controlled by Constants.Arm.BUNDT_ANGLE
    joystick.rightTrigger().whileTrue(new BundtShot(peterSubsystem, armSubsystem, joystickSubsystem));

    joystick.a().whileTrue(new FireTeleop(peterSubsystem, armSubsystem, driveTrain, joystickSubsystem, frontBackFunction, leftRightFunction, speedFunction, redAllianceSupplier));
    
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
    
    // Outtake - left shoulder
    joystick
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                    new RunCommand(
                        () -> {
                          peterSubsystem.reverseMechanism();
                        },
                        peterSubsystem),
                    ArmToAngleCmd.toNeutral(armSubsystem).withTolerance(1))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    // zero-heading - x
    joystick.x().onTrue(
        driveTrain.runOnce(() ->
            driveTrain.seedFieldRelative(
                new Pose2d(new Translation2d(1.34, 5.5),Rotation2d.fromDegrees(0)))));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(1);
  }
}
