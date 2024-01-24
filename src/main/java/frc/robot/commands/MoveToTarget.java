package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTarget extends Command {
    private Pose2d relativePose;
    private SwerveSubsystem swerve;
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    PathPlannerPath path;
    Command followThePath;

    public MoveToTarget(SwerveSubsystem swerve, Pose2d relativePose) {
        this.swerve = SwerveSubsystem.getInstance();
        this.relativePose = relativePose;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
      // Create a list of bezier points from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
      Pose2d currentPose = swerve.getState().Pose;
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        currentPose,
        new Pose2d(currentPose.getTranslation().plus(relativePose.getTranslation()), currentPose.getRotation().plus(relativePose.getRotation()))
      );

      // Create the path using the bezier points created above
      path = new PathPlannerPath(
            bezierPoints,
            constraints, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(currentPose.getRotation().plus(relativePose.getRotation()).getDegrees())) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );

      path.preventFlipping = true;

      SmartDashboard.putNumber("Follow the Path doesn't throw any errors", 0);
      followThePath = AutoBuilder.followPath(path);
      followThePath.initialize();
      SmartDashboard.putNumber("Follow the Path doesn't throw any errors", 1);
    }

    private int countExecutes = 0;
    public void execute() {
      if (followThePath != null) {
        followThePath.execute();
        SmartDashboard.putNumber("count executes", countExecutes);
        countExecutes++;
      }
      else {
        SmartDashboard.putNumber("count executes", -1);
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (followThePath != null)
            followThePath.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return followThePath != null ? followThePath.isFinished() : false;
    }
}
