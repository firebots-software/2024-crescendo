
package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.PeterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmRotateCommand extends Command {
private PeterSubsystem intake; 
  public ArmRotateCommand(PeterSubsystem intake) {
    this.intake = intake; 
    addRequirements(intake);
}



}

