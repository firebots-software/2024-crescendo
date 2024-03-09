package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
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
      double headingToleranceDegrees,
      double armToleranceDegrees,
      Supplier<Boolean> redSide,
      Supplier<Boolean> increaseAngle) {
    addCommands(
        new SpinUpShooter(peter),
        ArmToAngleCmd.aimAtSpeaker(arm, swerve, redSide, increaseAngle)
            .withTolerance(armToleranceDegrees),
        SwerveLockedAngleCmd.fromPoseMirrored(
                frontBackFunction,
                leftRightFunction,
                () -> Constants.Landmarks.Speaker.POSE.getTranslation(),
                speedFunction,
                swerve,
                redSide)
            .withToleranceEnd(headingToleranceDegrees));
  }

  // Constructs without a end condition
  public AimAtSpeaker(
      PeterSubsystem peter,
      ArmSubsystem arm,
      SwerveSubsystem swerve,
      Supplier<Double> frontBackFunction,
      Supplier<Double> leftRightFunction,
      Supplier<Double> speedFunction,
      Supplier<Boolean> redSide,
      Supplier<Boolean> increaseAngle) {
    this(
        peter,
        arm,
        swerve,
        frontBackFunction,
        leftRightFunction,
        speedFunction,
        -1,
        -1,
        redSide,
        increaseAngle);
  }
}
