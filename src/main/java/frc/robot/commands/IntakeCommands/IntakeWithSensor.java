// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Peter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeWithSensor extends Command {
private Peter intake; 
  public IntakeWithSensor(Peter intake) {
    this.intake = intake; 
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(0.5);
    if(intake.notePresent()) {
            intake.runIntake(0);
            intake.runPreShooter(0.5); // run 3 inches more
        }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.runIntake(0);

}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.notePresent();
   }
}
