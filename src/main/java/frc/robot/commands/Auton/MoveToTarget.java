package frc.robot.commands.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;
import java.util.List;
import java.util.function.Supplier;

public class MoveToTarget extends Command {
  private Pose2d[] absolutePoses;
  private SwerveSubsystem swerve;
  private Supplier<Boolean> reflected;
  private Rotation2d startRotation2d;
  PathPlannerPath constructedPath;
  Command pathCommand;

  private MoveToTarget(
      SwerveSubsystem swerve, Pose2d[] absolutePoses, Supplier<Boolean> reflected) {
    this.absolutePoses = absolutePoses;
    this.swerve = SwerveSubsystem.getInstance();
    this.reflected = reflected;
    addRequirements(swerve);
  }

  // private MoveToTarget(
  //   SwerveSubsystem swerve, Pose2d[] absolutePoses, Supplier<Boolean> reflected, Rotation2d
  // startRotation2d) {
  //   this.absolutePoses = absolutePoses;
  //   this.swerve = SwerveSubsystem.getInstance();
  //   this.reflected = reflected;
  //   this.startRotation2d=startRotation2d;
  //   addRequirements(swerve);
  // }

  private MoveToTarget(SwerveSubsystem swerve, Pose2d absolutePose, Supplier<Boolean> reflected) {
    this(swerve, new Pose2d[] {absolutePose}, reflected);
  }

  @Override
  public void initialize() {
    // constructing the list of path points using absolute coordinates on the field

    Pose2d currentPose = swerve.getState().Pose;
    // if(startRotation2d != null){
    //   currentPose = new Pose2d(currentPose.getTranslation(), startRotation2d);
    // }

    Pose2d[] poseArray = new Pose2d[absolutePoses.length + 1];
    poseArray[0] = currentPose;
    for (int i = 1; i < poseArray.length; i++) {
      poseArray[i] =
          (reflected.get()
              ? MiscUtils.reflectAcrossMidline(this.absolutePoses[i - 1])
              : this.absolutePoses[i - 1]);
    }

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poseArray);

    // create the path using the path points and constraints (also providing the final robot
    // heading)
    constructedPath =
        new PathPlannerPath(
            bezierPoints,
            Constants.Swerve.PPConstants.PATH_PLANNER_CONSTRAINTS,
            new GoalEndState(
                0.0, poseArray[poseArray.length - 1].getRotation()) // goal end velocity and heading
            );

    // prevent automatic path flipping by AutoBuilder (we want to execute absolute path)
    constructedPath.preventFlipping = true;

    // Command of the built auto path
    pathCommand = AutoBuilder.followPath(constructedPath);
    pathCommand.initialize();

    // SmartDashboard.putNumber("TranslationX", absolutePose.getX());
    // SmartDashboard.putNumber("TranslationY", absolutePose.getY());
    // SmartDashboard.putNumber("Rotation", absolutePose.getRotation().getDegrees());
    SmartDashboard.putString("ended", "nope");
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
    SmartDashboard.putString("ended", "yes");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand != null ? pathCommand.isFinished() : false;
  }

  // Factory pattern (two separate constructors that invoke the mirror / non mirror)
  public static Command withAbsolute(SwerveSubsystem swerve, Pose2d... absolutePose) {
    return new MoveToTarget(swerve, absolutePose, () -> false);
  }

  public static Command withMirror(
      SwerveSubsystem swerve, Supplier<Boolean> mirror, Pose2d... absolutePose) {
    return new MoveToTarget(swerve, absolutePose, mirror);
  }
}
