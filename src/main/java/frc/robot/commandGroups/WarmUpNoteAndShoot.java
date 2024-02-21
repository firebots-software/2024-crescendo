package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DebugCommands.Shooter;
import frc.robot.commands.PeterCommands.ShootNote;
import frc.robot.subsystems.PeterSubsystem;

public class WarmUpNoteAndShoot extends SequentialCommandGroup {
  public WarmUpNoteAndShoot(PeterSubsystem shooter) {
    addCommands(new Shooter(shooter).withTimeout(3), new ShootNote(shooter).withTimeout(3));
  }
}
