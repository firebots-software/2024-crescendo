package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;
import java.util.List;

public class MoveToTarget extends Command {
  private Pose2d absolutePose;
  private SwerveSubsystem swerve;

  PathPlannerPath constructedPath;
  Command pathCommand;

  private MoveToTarget(SwerveSubsystem swerve, Pose2d absolutePose, boolean reflected) {
    this.swerve = SwerveSubsystem.getInstance();
    this.absolutePose = (reflected) ? MiscUtils.reflectAcrossMidline(absolutePose) : absolutePose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // constructing the list of path points using absolute coordinates on the field
    Pose2d currentPose = swerve.getState().Pose;
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currentPose, absolutePose);

    // create the path using the path points and constraints (also providing the final robot
    // heading)
    constructedPath =
        new PathPlannerPath(
            bezierPoints,
            Constants.Swerve.PPConstants.PATH_PLANNER_CONSTRAINTS,
            new GoalEndState(0.0, absolutePose.getRotation()) // goal end velocity and heading
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
  public static Command withAbsolute(SwerveSubsystem swerve, Pose2d absolutePose) {
    return new MoveToTarget(swerve, absolutePose, false);
  }

  public static Command withMirror(SwerveSubsystem swerve, Pose2d absolutePose, boolean mirror) {
    return new MoveToTarget(swerve, absolutePose, mirror);
  }
}
