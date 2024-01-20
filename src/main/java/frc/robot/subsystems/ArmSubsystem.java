package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmSubsystem {

        private TalonFX r1, r2, l1, l2;
        private TalonFX master;
        private TrapezoidProfile profile;
        private TrapezoidProfile.Constraints tp;

        private double setPos;

        public ArmSubsystem(int portR1, int portR2, int portL1, int portL2){
            tp = new TrapezoidProfile.Constraints(10, 20);
            profile = new TrapezoidProfile(tp);
            CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withSupplyCurrentLimit(5.0);

            Slot0Configs s0c = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKG(0).withKV(0).withKA(0);

            r1 = new TalonFX(portR1);
            r2 = new TalonFX(portR2);
            l1 = new TalonFX(portL1);
            l2 = new TalonFX(portL2);

            Follower f = new Follower(portR1, false);
            r2.setControl(f);
            l1.setInverted(true);
            l1.setControl(f);
            l2.setInverted(true);
            l2.setControl(f);
            
            r1.getConfigurator().apply(clc);
            r2.getConfigurator().apply(clc);
            l1.getConfigurator().apply(clc);
            l2.getConfigurator().apply(clc);

            master = r1;
            master.getConfigurator().apply(s0c);
            setPos = master.getPosition().getValue();
        }
    
    public void setPosition(double setPos){
        this.setPos = setPos;
        toPosition();
    }

    public void toPosition(){
        TrapezoidProfile.State setPoint = new TrapezoidProfile.State(setPos, 0);
        TrapezoidProfile.State currentPoint = new TrapezoidProfile.State(master.getPosition().getValue(),master.getVelocity().getValue());

        setPoint = profile.calculate(profile.totalTime(), currentPoint, setPoint);
        PositionDutyCycle m_positionControl = new PositionDutyCycle(setPoint.position);
        m_positionControl.Position = setPoint.position;
        m_positionControl.Velocity = setPoint.velocity;
        master.setControl(m_positionControl);
    }

    // public double rotateArmToSpeakerPosition(double pos, double SpeakerHeight){
    //     return determineAngle(pos, SpeakerHeight);
    // }

    public double determineAngle(Pose2d pos, double height) {
        double x = pos.getX()*pos.getX();
        double y = pos.getY()*pos.getY();
        return java.lang.Math.atan(java.lang.Math.sqrt(x+y)/height);   
    }

    public Pose2d determineRobotPosition() {
        //vision stuff
        return pos;
    }

    public void goToArmHeight(double angle) {
        //motor stuff
    }
}


