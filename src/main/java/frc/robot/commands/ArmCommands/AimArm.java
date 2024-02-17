package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class AimArm extends Command {
  private ArmSubsystem armSubsystem;
  private PeterSubsystem peterSubsystem;

  public AimArm(ArmSubsystem armSubsystem, PeterSubsystem peterSubsystem) {
    this.armSubsystem = armSubsystem;
    this.peterSubsystem = peterSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (peterSubsystem.notePresent()) {
      armSubsystem.rotateToRestPosition();
    } else {
      armSubsystem.rotateArmToSpeakerPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // armSubsystem.rotateArmToRestPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
