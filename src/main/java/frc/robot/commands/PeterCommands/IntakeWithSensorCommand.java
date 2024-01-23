// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeWithSensorCommand extends Command {
  private PeterSubsystem peterSubsystem;

  public IntakeWithSensorCommand(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.runIntake(0.5);
    if (peterSubsystem.notePresent()) {
      peterSubsystem.runIntake(0);
      while (peterSubsystem.getPreShooterPosition() < 1024) { // whatever is 3 inches pls be right
        peterSubsystem.runPreShooter(0.5);
      }
    }
    peterSubsystem.runPreShooter(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return peterSubsystem.notePresent();
  }
}
