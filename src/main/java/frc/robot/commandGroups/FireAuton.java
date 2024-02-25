package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PeterCommands.ShootNoWarmup;
import frc.robot.commands.SwerveCommands.SwerveLockedAngleCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class FireAuton extends SequentialCommandGroup {
  public FireAuton(
      PeterSubsystem peterSubsystem,
      ArmSubsystem armSubsystem,
      SwerveSubsystem driveTrain,
      double tolerance,
      Supplier<Boolean> redside) {
    addCommands(
        new AimAtSpeaker(
                peterSubsystem,
                armSubsystem,
                driveTrain,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                tolerance,
                redside)
            .withTimeout(1.0),
        new ParallelCommandGroup(
            new ShootNoWarmup(peterSubsystem).withTimeout(0.5),

            // we need this a second time because the first one ended in the
            // aimBeforeShootCommand, this time without a tolerance end
            SwerveLockedAngleCmd.fromPoseMirrored(
                    () -> 0.0,
                    () -> 0.0,
                    () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                    () -> 0.0,
                    driveTrain,
                    redside)
                .withToleranceEnd(tolerance)));
  }
}
