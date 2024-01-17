package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Peter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class RunShooter extends Command {
private double shooterSpeed; 
private Peter shooter; 
  public RunShooter(Peter shooter) {
    this.shooter = shooter; 
    this.shooterSpeed = Constants.Intake.SHOOTER_SPEED;
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runShooter(shooterSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.runShooter(0);

}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
   }
}



