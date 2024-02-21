package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Shoot extends Command {
  private PeterSubsystem peterSubsystem;
  private SwerveSubsystem swerve;
  private ArmSubsystem arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param peterSubsystem The subsystem used by this command.
   */
  public Shoot(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinLeftShooter();
    peterSubsystem.spinRightShooter();
    peterSubsystem.spinUpPreShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopLeftShooter();
    peterSubsystem.stopRightShooter();
    peterSubsystem.stopPreShooterMotor();
    SmartDashboard.putNumber("Pose X", swerve.getState().Pose.getX());
    SmartDashboard.putNumber("Pose Y", swerve.getState().Pose.getY());
    SmartDashboard.putNumber("Pose Heading", swerve.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Arm Angle", arm.getCorrectedDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
