
package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmToDegree extends Command {
    private final ArmSubsystem armSubsystem; 

public ArmToDegree(ArmSubsystem armSubsystem, double angle) {
    this.armSubsystem = armSubsystem; 
    addRequirements(armSubsystem);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {

  
}

public double getArmToSpeaker(ArmSubsystem arm, double angle, Pose2d pos){
  double speakerHeight = Constants.FieldDimensions.speakerHeightMeters; // make a constant in Constants.java
  return angle = arm.determineAngle(pos, speakerHeight);
  

}

public double getArmToAmp(ArmSubsystem arm, double angle, Pose2d pos){
  double ampHeight = Constants.FieldDimensions.ampHeightMeters;
  return angle = arm.determineAngle(pos, ampHeight);

}

public double getArmToIntake(ArmSubsystem arm, double angle, Pose2d pos) {
  return angle = arm.determineAngle(pos, Constants.FieldDimensions.intakeModeHeightMeters);
}


// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false; 
 }




}

