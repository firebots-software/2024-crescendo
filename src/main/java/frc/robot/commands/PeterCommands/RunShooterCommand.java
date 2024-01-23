<<<<<<< HEAD:src/main/java/frc/robot/commands/PeterCommands/RunShooterCommand.java
package frc.robot.commands.PeterCommands;
=======
package frc.robot.commands.IntakeCommands;
>>>>>>> 72fb8899ac45ab31b52f977fe44e3d7ec39dba6a:src/main/java/frc/robot/commands/IntakeCommands/RunShooterCommand.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PeterSubsystem;

/** An example command that uses an example subsystem. */
public class RunShooterCommand extends Command {
  private double shooterSpeed;
  private PeterSubsystem shooter;

  public RunShooterCommand(PeterSubsystem shooter) {
    this.shooter = shooter;
    this.shooterSpeed = Constants.Intake.SHOOTER_SPEED;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
