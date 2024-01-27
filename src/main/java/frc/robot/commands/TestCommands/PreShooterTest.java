package frc.robot.commands.TestCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;


// delete in future!!!!!!!
public class PreShooterTest extends Command {
     private PeterSubsystem preShooter;

  public PreShooterTest(PeterSubsystem preShooter) {
    this.preShooter = preShooter;
    addRequirements(preShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    preShooter.movePreShooterMotorPosition(3); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    preShooter.stopPreShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

