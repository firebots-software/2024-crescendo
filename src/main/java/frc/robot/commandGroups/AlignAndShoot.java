package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.AlignArmToShoot;
import frc.robot.commands.SwerveCommands.RobotBaseAlignment;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignAndShoot extends SequentialCommandGroup {
  public AlignAndShoot(SwerveSubsystem swerve, ArmSubsystem arm, PeterSubsystem peter) {
      addCommands(
        new RobotBaseAlignment(swerve),
        new AlignArmToShoot(arm, swerve.getState().Pose)
      );
  }
}