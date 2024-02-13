package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TestEncoderSubsystem;
import java.util.function.Supplier;

public class SwerveTest extends Command{
    private TestEncoderSubsystem testEncoderSubsystem;
    private Supplier<Double> angleOffsetSupplier;
    public SwerveTest(TestEncoderSubsystem testEncoderSubsystem, Supplier<Double> angleOffsetSupplier){
        this.testEncoderSubsystem = testEncoderSubsystem;
        this.angleOffsetSupplier = angleOffsetSupplier;
        addRequirements(testEncoderSubsystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("running");
    SmartDashboard.putBoolean("SwerveTest running", isScheduled());
    double setPos = Constants.Swerve.FRONT_RIGHT.CANcoderOffset*360 + 60.0 + angleOffsetSupplier.get();
    double supplierVal = angleOffsetSupplier.get();
    SmartDashboard.putNumber("CANCODER OFFSET: ", Constants.Swerve.FRONT_RIGHT.CANcoderOffset);
    SmartDashboard.putNumber("SwerveTest pos", setPos);
    SmartDashboard.putNumber("SwerveTest supplier", supplierVal);
    SmartDashboard.putNumber("The PID value: ", supplierVal);
    testEncoderSubsystem.setTargetPosition(setPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testEncoderSubsystem.rotateToResetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // wait for x seconds
  }
}
