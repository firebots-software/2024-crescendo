package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.MoveToTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;

public class RobotBaseAlignment extends Command {
  private SwerveSubsystem swerve;
  private Pose2d robotPose;
  Command mt;

  public RobotBaseAlignment(SwerveSubsystem swerve) {
    this.swerve = SwerveSubsystem.getInstance();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotRotation = MiscUtils.getAlignmentBaseRotation(swerve.getState().Pose, Constants.Landmarks.SUBWOOFER_LOCATION_GROUND);
    mt = MoveToTarget.withAbsolute(swerve, new Pose2d(swerve.getState().Pose.getTranslation(), new Rotation2d(robotRotation)));
    mt.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mt.isFinished();
  }
}
