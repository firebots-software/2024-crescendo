package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;

public class MoveToTarget extends Command {
    private Pose2d absolutePose;
    private SwerveSubsystem swerve;

    PathPlannerPath constructedPath;
    Command pathCommand;

    private MoveToTarget(SwerveSubsystem swerve, Pose2d absolutePose, boolean reflected) {
        this.swerve = SwerveSubsystem.getInstance();
        if (reflected) {
          this.absolutePose = MiscUtils.reflectAcrossMidline(absolutePose);
        }
        else {
          this.absolutePose = absolutePose;
        }
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
      // Create a list of bezier points from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
      Pose2d currentPose = swerve.getState().Pose;
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        currentPose,
        absolutePose);
      
      // Create the path using the bezier points created above
      constructedPath = new PathPlannerPath(
            bezierPoints,
            Constants.PPConstants.constraints, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, absolutePose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );

      constructedPath.preventFlipping = true;

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
        if (pathCommand != null)
            pathCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return pathCommand != null ? pathCommand.isFinished() : false;
    }

    public static Command withAbsolute(SwerveSubsystem swerve, Pose2d absolutePose) {
      return new MoveToTarget(swerve, absolutePose, false);
    }

    public static Command withMirror(SwerveSubsystem swerve, Pose2d absolutePose, boolean mirror) {
      return new MoveToTarget(swerve, absolutePose, mirror);
    }
}
