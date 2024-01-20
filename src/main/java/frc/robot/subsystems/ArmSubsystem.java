package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommands.ArmRotateCommand;

public class ArmSubsystem extends SubsystemBase
{
    public void moveToRotations(double pos) {
        double currPos = position.getValue();
        preShooterMotor.setControl((preShooterMotorPIDRequest.withPosition(pos+currPos)));
    }

    public void rotateArmToSpeakerPosition() 
    {
        // get encoder values to know where arm is
        // using arm position move arm to speaker angle
    }
       
    public void rotateArmToAmpPosition() 
    {

    }

    public void rotateArmToRestPosition() 
    {
    }

    public Command nextState() {
        
        return new ArmRotateCommand(null, 0);
    }

    
}
