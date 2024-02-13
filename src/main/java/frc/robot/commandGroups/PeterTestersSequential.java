package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PeterCommands.ShootNote;
import frc.robot.commands.TestCommands.IntakeMotorTest;
import frc.robot.commands.TestCommands.PreShooterTest;
import frc.robot.subsystems.PeterSubsystem;

public class PeterTestersSequential extends SequentialCommandGroup {
  public PeterTestersSequential(
      PeterSubsystem intake, PeterSubsystem shooter, PeterSubsystem preShooter) {
    addCommands(
        new IntakeMotorTest(intake).withTimeout(3),
        new PreShooterTest(preShooter).withTimeout(3),
        new ShootNote(preShooter).withTimeout(3));
  }
}
