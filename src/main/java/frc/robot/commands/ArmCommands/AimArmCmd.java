package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimArmCmd extends Command {
  private ArmSubsystem armSubsystem;
  private SwerveSubsystem swerveSubsystem;

  public AimArmCmd(ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem) {
    this.armSubsystem = armSubsystem;
    this.swerveSubsystem = swerveSubsystem; // only used to get pose
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.rotateArmToSpeakerPosition(swerveSubsystem.getState().Pose.getTranslation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atTarget(1);
  }
}