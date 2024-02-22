package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PeterCommands.ShootNoWarmup;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.PeterSubsystem;

public class WarmUpNoteAndShoot extends SequentialCommandGroup {
  public WarmUpNoteAndShoot(PeterSubsystem shooter) {
    addCommands(
      new SpinUpShooter(shooter), new ShootNoWarmup(shooter));
  }
}
