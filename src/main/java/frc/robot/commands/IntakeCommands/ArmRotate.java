
package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Peter;
import edu.wpi.first.wpilibj2.command.Command;




public class ArmRotate extends Command {
private Peter intake; 
  public ArmRotate(Peter intake) {
    this.intake = intake; 
    addRequirements(intake);
}
}

