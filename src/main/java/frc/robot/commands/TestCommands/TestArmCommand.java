package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class TestArmCommand extends Command {
      private ArmSubsystem armSubsystem;
      private Supplier<Double> joystickSpeed;

      public TestArmCommand(Supplier<Double> joystickSpeed) {
        this.armSubsystem = armSubsystem;
        this.joystickSpeed = joystickSpeed;
      }


public TestArmCommand(ArmSubsystem armSubsystem2, Object joystickSpeed2) {
		//TODO Auto-generated constructor stub
	}


@Override
public void initialize() {

}

@Override
public void execute() {
    SmartDashboard.putNumber("Arm Values", 1.0);
    armSubsystem.setTargetPosition(1);
}

@Override
public void end(boolean interrupted) {
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}
}
