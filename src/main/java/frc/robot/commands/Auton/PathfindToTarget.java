package frc.robot.commands.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class PathfindToTarget extends Command {
  Pose2d targetPose;
  Command pathfindingCommand;

  private PathfindToTarget(SwerveSubsystem swerve, Supplier<Boolean> reflected, Pose2d targetPose) {
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            Constants.Swerve.PPConstants.PATH_PLANNER_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            100d // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
            );
    pathfindingCommand.initialize();
  }

  public void execute() {
    if (pathfindingCommand != null) {
      pathfindingCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathfindingCommand != null) {
      pathfindingCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return pathfindingCommand != null ? pathfindingCommand.isFinished() : false;
  }

  public static Command withAbsolute(SwerveSubsystem swerve, Pose2d targetPose) {
    return new PathfindToTarget(swerve, () -> false, targetPose);
  }

  public static Command withMirror(
      SwerveSubsystem swerve, Supplier<Boolean> mirror, Pose2d targetPose) {
    return new PathfindToTarget(swerve, mirror, targetPose);
  }
}
