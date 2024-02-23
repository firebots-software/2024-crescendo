package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveLockedAngleCmd extends SwerveJoystickCommand {

  private static final PIDController turningPID = new PIDController(1d, 0.002, 0.01d);
  private static final double MAX_RATE = 0.4; // Stick command output
  private final Supplier<Double> error;
  private double tolerance = -1d;

  public SwerveLockedAngleCmd(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Rotation2d> turnTarget,
      Supplier<Double> speedControlFunction,
      SwerveSubsystem swerveSubsystem) {
    super(
        frontBackFunction,
        leftRightFunction,
        () -> {
          // Rotate 90 degrees so we avoid all wrap regions
          Rotation2d actualTarget = turnTarget.get().rotateBy(Rotation2d.fromDegrees(90));
          Rotation2d computedError = actualTarget.minus(getSwerveRotation(swerveSubsystem));
          double computedRotation = turningPID.calculate(computedError.getRadians());
          computedRotation = MathUtil.clamp(computedRotation, -MAX_RATE, MAX_RATE);
          if (Math.abs(computedError.getDegrees()) < 1) {
            computedRotation = 0;
          }

          // Logging
          SmartDashboard.putNumber("Corrected Target", turnTarget.get().getDegrees());
          SmartDashboard.putNumber("Shooter Angular Error", computedError.getDegrees());
          SmartDashboard.putNumber("Shooter Computed Output", computedRotation);
          SmartDashboard.putNumber(
              "Swerve Heading", getSwerveRotation(swerveSubsystem).getDegrees());

          return -computedRotation;
        },
        speedControlFunction,
        swerveSubsystem);

    // bro this be really bad coding, aaaa ~java~ OOP is stupid
    error =
        () -> turnTarget.get().minus(swerveSubsystem.getState().Pose.getRotation()).getDegrees();
  }

  @Override
  public void initialize() {
    super.initialize();
    turningPID.reset();
  }

  private static Rotation2d getSwerveRotation(SwerveSubsystem subsystem) {
    return subsystem.getState().Pose.getRotation().rotateBy(Rotation2d.fromDegrees(90));
  }

  /**
   * Creates a swerve locked angle command that locks the angle of the scoring side of the robot
   * (the butt) to the desired pose.
   *
   * @param frontBackFunction
   * @param leftRightFunction
   * @param pointAtFunction the translation to point at, with 0, 0 defined as the field origin.
   * @param speedControlFunction
   * @param swerveSubsystem
   * @return
   */
  public static SwerveLockedAngleCmd fromPose(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Translation2d> pointAtFunction,
      Supplier<Double> speedControlFunction,
      SwerveSubsystem swerveSubsystem) {
    return new SwerveLockedAngleCmd(
        frontBackFunction,
        leftRightFunction,
        () ->
            pointAtFunction
                .get()
                .minus(swerveSubsystem.getState().Pose.getTranslation())
                .rotateBy(
                    Rotation2d.fromRadians(
                        Math.PI)) // so that the scoring side/butt is facing the target
                .getAngle(),
        speedControlFunction,
        swerveSubsystem);
  }

  /**
   * Adds a rotation tolerance so that this command can end. Note the command will never finish by
   * itself if this is not set.
   *
   * @param tolerance The tolerance, in radians, that the robot heading must be in for the command
   *     to end. If negative, the command will not end by itself.
   * @return Itself
   */
  public SwerveLockedAngleCmd withToleranceEnd(double tolerance) {
    this.tolerance = tolerance;
    return this;
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("swerve locked angle error", error.get());
    SmartDashboard.putNumber("swerve locked angle tolerance", tolerance);
    return Math.abs(error.get()) < tolerance;
  }
}
