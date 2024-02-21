package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DebugCommands.Shooter;
import frc.robot.commands.PeterCommands.ShootNote;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class WarmUpNoteAndShoot extends SequentialCommandGroup {
  public WarmUpNoteAndShoot(PeterSubsystem shooter, SwerveSubsystem swerveSubsystem) {
    addCommands(
        new Shooter(shooter, swerveSubsystem).withTimeout(3),
        new ShootNote(shooter, swerveSubsystem).withTimeout(3));
  }
}
