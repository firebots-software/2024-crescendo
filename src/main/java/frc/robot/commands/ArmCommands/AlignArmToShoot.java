package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.MiscUtils;

public class AlignArmToShoot extends Command {
  private ArmSubsystem armSubsystem;
  private Pose2d robotPose;
  public AlignArmToShoot(ArmSubsystem armSubsystem, Pose2d robotPose) {
    this.armSubsystem = armSubsystem;
    this.robotPose = robotPose;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d armAngleToSet = MiscUtils.getAlignmentArmRotation(robotPose);
    armSubsystem.setTargetDegrees(armAngleToSet.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.rotateToRestPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
