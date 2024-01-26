package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class rightTrigger extends SequentialCommandGroup {
  public rightTrigger(boolean isLeftTriggerPressed) {
    if (isLeftTriggerPressed) {

      addCommands();
    }
  }
}

/*package frc.robot.commandGroups;

import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.commands.RunShooterWheelsCommand;
import frc.robot.commands.ShootNoteCommand;

public class rightTrigger extends SequentialCommandGroup {
    public rightTrigger(boolean warmed) {
        if (warmed) {
            addCommands(
                new RunConveyorCommand(),
                new ShootNoteCommand()
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new ArmRotateCommand(speakerPosition),
                    new RunShooterWheelsCommand(),
                    new MoveArmCommand()
                ),
                new SequentialCommandGroup(
                    new RunConveyorCommand(),
                    new ShootNoteCommand()
                )
            );
        }
    }

    // You need to define the speaker position or retrieve it from somewhere
    private double speakerPosition = 0.0; // Example value, should be adjusted
} */
