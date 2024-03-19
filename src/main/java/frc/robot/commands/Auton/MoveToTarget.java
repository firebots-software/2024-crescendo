package frc.robot.commands.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;
import java.util.List;
import java.util.function.Supplier;

public class MoveToTarget extends Command {
  private Pose2d[] pointDirsInput, pointDirsConvert;
  private SwerveSubsystem swerve;
  private Supplier<Boolean> reflected;
  private Rotation2d endRotation;
  private double goalEndVelo;
  PathPlannerPath constructedPath;
  Command pathCommand;

  private MoveToTarget(
      SwerveSubsystem swerve,
      Pose2d[] pointDirs,
      Supplier<Boolean> reflected,
      Rotation2d endRotation,
      double goalEndVelo) {
    this.pointDirsInput = pointDirs;
    this.swerve = SwerveSubsystem.getInstance();
    this.reflected = reflected;
    this.endRotation = (reflected.get()) ? new Rotation2d(Math.PI - endRotation.getRadians()) : endRotation;
    this.goalEndVelo = goalEndVelo;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pointDirsConvert[0] = swerve.getState().Pose;
    for (int i = 0; i < pointDirsInput.length; i++) {
      pointDirsConvert[i+1] =
          (reflected.get() ? MiscUtils.reflectAcrossMidline(pointDirsInput[i]) : pointDirsInput[i]);
    }

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(pointDirsConvert);

    // create the path using the path points and constraints (also providing the final robot
    // heading)
    constructedPath =
        new PathPlannerPath(
            bezierPoints,
            Constants.Swerve.PPConstants.PATH_PLANNER_CONSTRAINTS,
            new GoalEndState(goalEndVelo, endRotation) // goal end velocity and heading
            );

    // prevent automatic path flipping by AutoBuilder (we want to execute absolute path)
    constructedPath.preventFlipping = true;

    // Command of the built auto path
    pathCommand = AutoBuilder.followPath(constructedPath);
    pathCommand.initialize();
  }

  public void execute() {
    if (pathCommand != null) {
      pathCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand != null ? pathCommand.isFinished() : false;
  }

  // Factory pattern (two separate constructors that invoke the mirror / non mirror)
  public static Command withAbsolute(
      SwerveSubsystem swerve, Rotation2d endRotation, Pose2d... pointDirs) {
    return new MoveToTarget(swerve, pointDirs, () -> false, endRotation, 0d);
  }

  public static Command withAbsolute(
      SwerveSubsystem swerve, Rotation2d endRotation, double goalEndVelo, Pose2d... pointDirs) {
    return new MoveToTarget(swerve, pointDirs, () -> false, endRotation, goalEndVelo);
  }

  public static Command withMirror(
      SwerveSubsystem swerve,
      Supplier<Boolean> mirror,
      Rotation2d endRotation,
      Pose2d... pointDirs) {
    return new MoveToTarget(swerve, pointDirs, mirror, endRotation, 0d);
  }

  public static Command withMirror(
      SwerveSubsystem swerve,
      Supplier<Boolean> mirror,
      Rotation2d endRotation,
      double goalEndVelo,
      Pose2d... pointDirs) {
    return new MoveToTarget(swerve, pointDirs, mirror, endRotation, goalEndVelo);
  }
}

