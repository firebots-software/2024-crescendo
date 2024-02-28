package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmToNeutralCmd;
import frc.robot.commands.ArmCommands.ArmToPickupCmd;
import frc.robot.commands.DebugCommands.Rumble;
import frc.robot.commands.PeterCommands.BackupPeter;
import frc.robot.commands.PeterCommands.RunIntakeUntilDetection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class Intake extends SequentialCommandGroup {
  public Intake(PeterSubsystem peter, ArmSubsystem arm, JoystickSubsystem joystick) {
    addCommands(
        new ParallelCommandGroup(new ArmToPickupCmd(arm), new RunIntakeUntilDetection(peter)),
        new ParallelCommandGroup(
            new ArmToNeutralCmd(arm),
            new BackupPeter(peter),
            Rumble.withNoBlock(joystick, 0.25, 0.5, 0)));
  }
}
