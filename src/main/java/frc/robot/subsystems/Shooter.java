package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private double wheelSpeed;
    private static Shooter instance;
    

    public Shooter() {

    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public void runFlywheels(double speed) {

    }

    public void stopFlywheels() {

    }

    public double getFlywheelSpeed() {
        return wheelSpeed;
    }
    
}
