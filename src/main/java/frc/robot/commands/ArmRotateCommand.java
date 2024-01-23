package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private double angle;

  public ArmRotateCommand(ArmSubsystem armSubsystem, double angle) {
    this.angle = angle;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setPosition(angle);
  }

  public double getAngleToSpeaker(ArmSubsystem arm, double angle, Pose2d pos) {
    double speakerHeight =
        Constants.FieldDimensions.SPEAKER_HEIGHT_METERS; // make a constant in Constants.java
    return angle = arm.determineAngle(pos, speakerHeight);
  }

  public double getArmToAmp(ArmSubsystem arm, double angle, Pose2d pos) {
    double ampHeight = Constants.FieldDimensions.AMP_HEIGHT_METERS;
    return angle = arm.determineAngle(pos, ampHeight);
  }

  public double getArmToIntake(ArmSubsystem arm, double angle, Pose2d pos) {
    double intakeHeight = Constants.FieldDimensions.INTAKE_MODE_HEIGHT_METERS;
    return angle = arm.determineAngle(pos, intakeHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
