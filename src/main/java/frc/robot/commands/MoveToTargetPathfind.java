package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;

public class MoveToTargetPathfind extends Command {
  private Pose2d absolutePose;
  private SwerveSubsystem swerve;

  PathPlannerPath constructedPath;
  Command pathCommand;

  private MoveToTargetPathfind(SwerveSubsystem swerve, Pose2d absolutePose, boolean reflected) {
    this.swerve = SwerveSubsystem.getInstance();
    if (reflected) {
      this.absolutePose = MiscUtils.reflectAcrossMidline(absolutePose);
    } else {
      this.absolutePose = absolutePose;
    }
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // // constructing the list of path points using absolute coordinates on the field
    // Pose2d currentPose = swerve.getState().Pose;
    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currentPose,
    // absolutePose);

    // // create the path using the path points and constraints (also providing the final robot
    // // heading)
    // constructedPath =
    //     new PathPlannerPath(
    //         bezierPoints,
    //         Constants.PPConstants.PATH_PLANNER_CONSTRAINTS,
    //         new GoalEndState(0.0, absolutePose.getRotation()) // goal end velocity and heading
    //         );

    // // prevent automatic path flipping by AutoBuilder (we want to execute absolute path)
    // constructedPath.preventFlipping = true;

    // // Command of the built auto path
    // pathCommand = AutoBuilder.followPath(constructedPath);
    // pathCommand.initialize();
    // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation
    // Create the constraints to use while pathfinding

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // SmartDashboard.putNumber("X", AutoBuilder)

    Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

// Create the constraints to use while pathfinding
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

// See the "Follow a single path" example for more info on what gets passed here
// SmartDashboard.putData("Pathfind to Pickup Pos", 
//     SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
//       new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
//       new PathConstraints(
//         4.0, 4.0, 
//         Units.degreesToRadians(360), Units.degreesToRadians(540)
//       ), 
//       0, 
//       0
//     ));
    pathCommand = AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    );

// Command pathfindingCommand = new PathfindHolonomic(
//         targetPose,
//         constraints,
//         0.0, // Goal end velocity in m/s. Optional
//         swerve::getPose,
//         swerve::getRobotRelativeSpeeds,
//         swerve::driveRobotRelative,
//         Constants.PPConstants.hpfc, // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
//         0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
//         swerve // Reference to drive subsystem to set requirements
// );

    // Command pathfindingCommand = new PathfindHolonomic(
    //     absolutePose,
    //     Constants.PPConstants.PATH_PLANNER_CONSTRAINTS,
    //     0.0, // Goal end velocity in m/s. Optional
    //     swerve::getPose,
    //     swerve::getRobotRelativeSpeeds,
    //     swerve::driveRobotRelative,
    //     Constants.PPConstants.hpfc, // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
    //     0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
    //     // swerve // Reference to drive subsystem to set requirements
    //   );
    // pathCommand =
    //     AutoBuilder.pathfindToPose(
    //         absolutePose,
    //         Constants.PPConstants.PATH_PLANNER_CONSTRAINTS
    //         swerve.getPose(),
    //         swerve.getCurrentRobotChassisSpeeds(),
    //         swerve.driveRobotRelative,
    //         0.0, // Goal end velocity in meters/sec
    //         0.0 // Rotation delay distance in meters. This is how far the robot should travel before
    //         // // attempting to rotate.
    //         );
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
    SmartDashboard.putBoolean("INTERRUPTED", interrupted);
    if (pathCommand != null) pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("FINISHED", pathCommand != null ? pathCommand.isFinished() : false);
    return pathCommand != null ? pathCommand.isFinished() : false;
  }

  // Factory pattern (two separate constructors that invoke the mirror / non mirror)
  public static Command withAbsolute(SwerveSubsystem swerve, Pose2d absolutePose) {
    return new MoveToTargetPathfind(swerve, absolutePose, false);
  }

  public static Command withMirror(SwerveSubsystem swerve, Pose2d absolutePose, boolean mirror) {
    return new MoveToTargetPathfind(swerve, absolutePose, mirror);
  }
}
