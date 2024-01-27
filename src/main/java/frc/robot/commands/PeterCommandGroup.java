package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.ArmRotateCommand;
import frc.robot.commands.PeterCommands.Shooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import java.util.function.Supplier;

public class PeterCommandGroup extends ParallelCommandGroup {
  public PeterCommandGroup(
      Supplier<Double> RTrigger,
      Supplier<Double> LTrigger,
      PeterSubsystem peter,
      ArmSubsystem arm) {
    boolean right = RTrigger.get() > 0;
    boolean left = LTrigger.get() > 0;
    addCommands(new ArmRotateCommand(arm));

    if (right) {
      if (peter.notePresent()) {
        addCommands(new Shooter(peter));
      } else {
        addCommands(new Intake(peter));
      }
      // If right is true, drop arm to intake level, else raies to speaker height
    }
    if (left) {
      addCommands(new WarmUpShooter(peter));
    }
    // left is used to aim and warm-up the mtoorrs

  }
}
