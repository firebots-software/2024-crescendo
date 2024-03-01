package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.DebugCommands.Rumble;
import frc.robot.commands.PeterCommands.ShootNoWarmup;
import frc.robot.commands.SwerveCommands.SwerveLockedAngleCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class FireTeleop extends SequentialCommandGroup {
  public FireTeleop(
      PeterSubsystem peterSubsystem,
      ArmSubsystem armSubsystem,
      SwerveSubsystem driveTrain,
      JoystickSubsystem joystickSubsystem,
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> speedFunction,
      Supplier<Boolean> redside) {
    addCommands(
        new AimAtSpeaker(
            peterSubsystem,
            armSubsystem,
            driveTrain,
            frontBackFunction,
            leftRightFunction,
            speedFunction,
            5,
            1,
            redside),
        new ParallelCommandGroup(
            new ShootNoWarmup(peterSubsystem, false).withTimeout(1),
            Rumble.withNoBlock(joystickSubsystem, 1, 1, 0.25),
            ArmToAngleCmd.aimAtSpeaker(armSubsystem, driveTrain, redside),
            SwerveLockedAngleCmd.fromPoseMirrored(
                frontBackFunction,
                leftRightFunction,
                () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                speedFunction,
                driveTrain,
                redside),
            new PrintCommand("FIRE TELEOP")));
  }
}
