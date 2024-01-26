// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class ArmAndPeterCommand extends Command {
  private PeterSubsystem peter;
  private ArmSubsystem arm;
  private Supplier<Double> RTrigger;
  private Supplier<Double> LTrigger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmAndPeterCommand(
      Supplier<Double> RTrigger,
      Supplier<Double> LTrigger,
      PeterSubsystem peter,
      ArmSubsystem arm) {
    this.RTrigger = RTrigger;
    this.LTrigger = LTrigger;
    this.peter = peter;
    this.arm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, peter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean right = RTrigger.get() > 0;
    boolean left = LTrigger.get() > 0;

    if (right) {
      peter.spinUpShooter();
      if (aim() && peter.isShooterReady()) {
        peter.moveNoteToShooter();
      }
    }
    if (left) {
      if (peter.notePresent()) {
        arm.rotateArmToSpeakerPosition();
        peter.spinUpShooter();
      } else {
        arm.rotateArmToRestPosition();
        peter.spinUpIntake();
      }
    } else {
      peter.stopIntake();
    }
    if (left == false && right == false) {
      peter.stopShooter();
    }
  }

  public boolean aim() {
    // TODO: Add logic to rotate the robot to the proper angle
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { // Should not be called, but if it is, then stop everything
    peter.stopIntake();
    peter.stopShooter();
    peter.stopPreShooterMotor();
    arm.rotateArmToRestPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // wait for x seconds
  }
}
