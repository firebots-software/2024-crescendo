package frc.robot.commands.TestCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class TestArmCommand extends Command {
      private ArmSubsystem armSubsystem;
      private Supplier<Double> joystickSpeed;

      public TestArmCommand(ArmSubsystem armSubsystem, Supplier<Double> joystickSpeed) {
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
    //SmartDashboard.putNumber("Arm Values", 1.0);
    armSubsystem.setTargetDegrees(joystickSpeed.get());
}

@Override
public void end(boolean interrupted) {
  armSubsystem.rotateArmToRestPosition();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}
}
