package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveTargetCmd extends SwerveJoystickCommand {

  private static final PIDController turningPID = new PIDController(3d, 0d, 0d);
  private final Supplier<Double> error;

  public SwerveTargetCmd(
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Pose2d> pointAtFunction,
      Supplier<Double> speedControlFunction,
      SwerveSubsystem swerveSubsystem) {
    super(
        frontBackFunction,
        leftRightFunction,
        () -> {
          return turningPID.calculate(
                  pointAtFunction
                      .get()
                      .relativeTo(swerveSubsystem.getState().Pose)
                      .getRotation()
                      .getRadians())
              / Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;
        }, // total bakwas
        speedControlFunction,
        swerveSubsystem);

    error =
        () ->
            pointAtFunction
                .get()
                .relativeTo(swerveSubsystem.getState().Pose)
                .getRotation()
                .getRadians();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(error.get()) < 0.02;
  }
}
