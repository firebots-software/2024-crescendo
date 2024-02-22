package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveLockedAngleCmd extends SwerveJoystickCommand {

  private static final PIDController turningPID = new PIDController(0.55d, 0d, 0d);
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
          return turningPID.calculate(
                  turnTarget
                      .get()
                      .minus(
                          swerveSubsystem
                              .getState()
                              .Pose
                              .getRotation()
                              .rotateBy(new Rotation2d(Math.PI)))
                      .getRadians())
              / Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;
        }, // total bakwas
        speedControlFunction,
        swerveSubsystem);

    turningPID.enableContinuousInput(-Math.PI, Math.PI);

    // bro this be really bad coding, aaaa ~java~ OOP is stupid
    error =
        () -> turnTarget.get().minus(swerveSubsystem.getState().Pose.getRotation()).getRadians();
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
                    new Rotation2d(Math.PI)) // so that the scoring side/butt is facing the target
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
    return Math.abs(error.get()) < tolerance;
  }
}
