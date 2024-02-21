package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.AimArmCmd;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.commands.SwerveCommands.SwerveLockedAngleCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class AimAtSpeaker extends ParallelCommandGroup {
  public AimAtSpeaker(
      PeterSubsystem peter,
      ArmSubsystem arm,
      SwerveSubsystem swerve,
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> speedFunction,
      double tolerance) {
    addCommands(
        new SpinUpShooter(peter),
        new AimArmCmd(arm, swerve),
        SwerveLockedAngleCmd.fromPose(
                frontBackFunction,
                leftRightFunction,
                () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                speedFunction,
                swerve)
            .withToleranceEnd(tolerance)); // don't want to end while in aim, so no tolerance
  }

  // Constructs without a end condition
  public AimAtSpeaker(
      PeterSubsystem peter,
      ArmSubsystem arm,
      SwerveSubsystem swerve,
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> speedFunction) {
    this(peter, arm, swerve, frontBackFunction, leftRightFunction, speedFunction, -1);
  }
}