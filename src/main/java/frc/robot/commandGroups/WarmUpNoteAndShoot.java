package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DebugCommands.Shooter;
import frc.robot.commands.PeterCommands.ShootNote;
import frc.robot.subsystems.PeterSubsystem;

public class WarmUpNoteAndShoot extends SequentialCommandGroup {
  public WarmUpNoteAndShoot(
      PeterSubsystem shooter, PeterSubsystem preShooter) {
    addCommands(
        new Shooter(shooter).withTimeout(3),
        new ShootNote(shooter, preShooter).withTimeout(3));
  }
} 
