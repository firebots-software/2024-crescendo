package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class PeterCommandGroup extends ParallelCommandGroup {
  public PeterCommandGroup(PeterSubsystem peter, ArmSubsystem arm) {
    addCommands(new ArmRotateCommand(arm));
    if (peter.notePresent()) {
      addCommands(new Shooter(peter));
    } else {
      addCommands(new Intake(peter));
    }
  }
}
