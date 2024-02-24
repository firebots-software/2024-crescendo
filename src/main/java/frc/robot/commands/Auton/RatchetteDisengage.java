package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RatchetteDisengage extends Command {

  private ArmSubsystem armSubsystem;
  private double initialAngle;

  public RatchetteDisengage(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    this.initialAngle = armSubsystem.getCorrectedDegrees();
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setTargetDegrees(initialAngle + 20.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atTarget(1);
  }
}
