package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToFinishClimbCmd extends Command {

private final ArmSubsystem arm;
private final Supplier<Double> angle;

  public ArmToFinishClimbCmd(Supplier<Double> angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setTargetDegrees(15);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.atTarget(1);
  }
    
}
