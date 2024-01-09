package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class StageSubsystem extends SubsystemBase{

    private StageSubsystem stageInstance;
    private TalonFX holderMotor;
    private TalonFX conveyorMotor;

    private StageSubsystem(){

    }

    public StageSubsystem getInstance(){
        if(stageInstance == null){
            stageInstance = new StageSubsystem();
        }
        return stageInstance;
    }
    
    public boolean notePresent(){
        return false;
    }
    
    public void turnOnHolder(){}
    public void turnOffHolder(){}
    public void turnOnConveyor(){}
    public void turnOffConveyor(){}
    
    
}
