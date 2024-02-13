// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// delete in future
package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

/** An example command that uses an example subsystem. */
public class PeterTest extends Command {
  private PeterSubsystem peterSubsystem;

  /**
   * Tests the intake by spinning them up and stopping them
   *
   * @param peterSubsystem The subsystem used by this command.
   */
  public PeterTest(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.getBoolean(this.toString(), true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinUpIntake();
    peterSubsystem.movePreShooterMotorPosition(3);
    peterSubsystem.spinUpShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopIntake();
    peterSubsystem.stopPreShooterMotor();
    peterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // wait for x seconds
  }
}
