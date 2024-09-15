package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;

public class LoggedTalonFX extends TalonFX{

    private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
    private String name;

    public LoggedTalonFX(String deviceName,int deviceId, String canbus) {
        super(deviceId, canbus);
        init();
        name = deviceName;
    }

        public LoggedTalonFX(String deviceName,int deviceId) {
        super(deviceId);
        init();
        name = deviceName;
    }

    public LoggedTalonFX(int deviceId, String canbus) {
        super(deviceId, canbus);
        init();
        name = "motor "+deviceId;
    }

    public LoggedTalonFX(int deviceId) {
        super(deviceId);
        init();
        name = "motor "+deviceId;
    }

    public void init(){
        motors.add(this);
    }

    public static void peroidic(){
        for(LoggedTalonFX l: motors){
            l.periodic();
        }
    }
    
    public void periodic(){
        DogLog.log(name + "/temperature(degC)",this.getDeviceTemp().getValue());
        DogLog.log(name + "/closedLoopError",this.getClosedLoopError().getValue());
        DogLog.log(name + "/closedLoopReference",this.getClosedLoopReference().getValue());

        DogLog.log(name + "/position(rotations)",this.getPosition().getValue());
        DogLog.log(name + "/velocity(rps)",this.getVelocity().getValue());
        DogLog.log(name + "/closedLoopReference(rps2)",this.getAcceleration().getValue());

        //Current
        DogLog.log(name + "/current/supply(A)",this.getSupplyCurrent().getValue());
        DogLog.log(name + "/current/stator(A)",this.getStatorCurrent().getValue());
        DogLog.log(name + "/current/torque(A)",this.getTorqueCurrent().getValue());

        //Voltage
        DogLog.log(name + "/voltage/motor(V)",this.getMotorVoltage().getValue());
        DogLog.log(name + "/voltage/motor(V)",this.getSupplyVoltage().getValue());
    }

}
