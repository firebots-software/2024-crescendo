package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class ArmRotateCommand extends Command {
  private ArmSubsystem armSubsystem;

  public ArmRotateCommand(ArmSubsystem armSubsystem, double angle) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  public ArmRotateCommand(PeterSubsystem peter) {
    // TODO Auto-generated constructor stub
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.rotateArmToSpeakerPosition();
  }

  // Called once the command ends or is interrupted.
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