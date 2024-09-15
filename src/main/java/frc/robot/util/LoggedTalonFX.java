package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;

public class LoggedTalonFX extends TalonFX{

    private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
    private String name;
    private String temperature,closedLoopError,closedLoopReference,position,velocity,acceleration,supplycurrent,statorcurrent,torquecurrent,motorvoltage,supplyvoltage;

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
        this.temperature=name + "/temperature(degC)";
        this.closedLoopError = name + "/closedLoopError";
        this.closedLoopReference = name + "/closedLoopReference";
        this.position=name + "/position(rotations)";
        this.velocity=name + "/velocity(rps)";
        this.acceleration=name + "/acceleration(rps2)";
        this.supplycurrent=name + "/current/supply(A)";
        this.statorcurrent=name + "/current/stator(A)";
        this.torquecurrent=name + "/current/torque(A)";
        this.motorvoltage=name + "/voltage/motor(V)";
        this.supplyvoltage=name + "/voltage/supply(V)";
    }

    public static void peroidic(){
        for(LoggedTalonFX l: motors){
            l.periodic();
        }
    }
    
    public void periodic(){
        DogLog.log(temperature,this.getDeviceTemp().getValue());
        DogLog.log(closedLoopError,this.getClosedLoopError().getValue());
        DogLog.log(closedLoopReference,this.getClosedLoopReference().getValue());

        DogLog.log(position,this.getPosition().getValue());
        DogLog.log(velocity,this.getVelocity().getValue());
        DogLog.log(acceleration,this.getAcceleration().getValue());

        //Current
        DogLog.log(supplycurrent,this.getSupplyCurrent().getValue());
        DogLog.log(statorcurrent,this.getStatorCurrent().getValue());
        DogLog.log(torquecurrent,this.getTorqueCurrent().getValue());

        //Voltage
        DogLog.log(motorvoltage,this.getMotorVoltage().getValue());
        DogLog.log(supplyvoltage,this.getSupplyVoltage().getValue());
    }

}
