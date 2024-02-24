package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Rumble extends Command {
  private GenericHID joystick;
  private final double intensity;

  public Rumble(GenericHID joystick, double intensity) {
    this.joystick = joystick;
    this.intensity = intensity;
  }

  @Override
  public void initialize() {
    joystick.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  public static Command withNoBlock(
      GenericHID joystick, double intensity, double timeoutSeconds, double delaySeconds) {
    return new InstantCommand(
        () ->
            CommandScheduler.getInstance()
                .schedule(
                    new WaitCommand(delaySeconds)
                        .andThen(new Rumble(joystick, intensity).withTimeout(timeoutSeconds))));
  }
}
