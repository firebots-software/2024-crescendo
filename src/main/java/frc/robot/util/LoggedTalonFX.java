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
        int id = this.getDeviceID();
        addLoggedValue(name,"temperature",this.getDeviceTemp());
        addLoggedValue(name,"closedLoopError",this.getClosedLoopError());
        addLoggedValue(name,"closedLoopReference",this.getClosedLoopReference());

        addLoggedValue(name,"position",this.getPosition());
        addLoggedValue(name,"velocity",this.getVelocity());
        addLoggedValue(name,"acceleration",this.getAcceleration());

        //Current
        addLoggedValue(name+"/current","supply",this.getSupplyCurrent());
        addLoggedValue(name+"/current","stator",this.getStatorCurrent());
        addLoggedValue(name+"/current","torque",this.getTorqueCurrent());

        //Voltage
        addLoggedValue(name+"/voltage","motor",this.getMotorVoltage());
        addLoggedValue(name+"/voltage","supply",this.getSupplyVoltage());
        
    }
    public void addLoggedValue(String path, String name, StatusSignal<Double> value){
        DogLog.log(path+"/"+name+"("+value.getUnits()+")",value.getValue());
    }


}
