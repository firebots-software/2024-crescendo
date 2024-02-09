package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class TestArmCommand extends Command {
      private ArmSubsystem armSubsystem;
      private Supplier<Double> joystickSpeed;

      public TestArmCommand(ArmSubsystem armSubsystem, Supplier<Double> joystickSpeed) {
        this.armSubsystem = armSubsystem;
        this.joystickSpeed=joystickSpeed;
      }
      
    
@Override
public void initialize() {

}

  @Override
  public void execute() {
      SmartDashboard.putNumber("Arm Values", 1.0);

  }
}
