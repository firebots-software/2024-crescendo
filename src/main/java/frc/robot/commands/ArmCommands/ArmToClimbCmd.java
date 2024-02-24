package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmToClimbCmd extends Command {

  private final ArmSubsystem arm;
  private final Supplier<Double> angle;

  public ArmToClimbCmd(Supplier<Double> angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setTargetDegrees(90);
  }

  @Override
  public void end(boolean interrupted) { 
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(1);
  }
}