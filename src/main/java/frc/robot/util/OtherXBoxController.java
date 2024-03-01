package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class OtherXBoxController extends XboxController {
    final Trigger a;
    final Trigger b;
    final Trigger x;
    final Trigger y;
    final Trigger leftBump;
    final Trigger rightBump;
    double threshold;

    public OtherXBoxController(int port) {
        super(port);
        a = new JoystickButton(this, Constants.OI.XBoxButtonID.A.value);
        b = new JoystickButton(this, Constants.OI.XBoxButtonID.B.value);
        x = new JoystickButton(this, Constants.OI.XBoxButtonID.X.value);
        y = new JoystickButton(this, Constants.OI.XBoxButtonID.Y.value);
        leftBump = new JoystickButton(this, Constants.OI.XBoxButtonID.LeftBumper.value);
        rightBump = new JoystickButton(this, Constants.OI.XBoxButtonID.RightBumper.value);
        threshold = 0.5;
        //TODO Auto-generated constructor stub
    }

    public Trigger a(){
        return a;
    }

    public Trigger b(){
        return b;
    }

    public Trigger x(){
        return x;
    }

    public Trigger y(){
        return y;
    }

    public Trigger leftBumper(){
        return leftBump;
    }

    public Trigger rightBumper(){
        return rightBump;
    }

    public double getLeftY(){
        return this.getRawAxis(Constants.OI.AxisID.LeftY.value);
    }

    public double getLeftX(){
        return this.getRawAxis(Constants.OI.AxisID.LeftX.value);
    }

    public double getRightX(){
        return this.getRawAxis(Constants.OI.AxisID.RightX.value);
    }
    public double getRightY(){
        return this.getRawAxis(Constants.OI.AxisID.RightY.value);
    }

    public Trigger leftTrigger(){
        return new Trigger(() -> this.getRawAxis(Constants.OI.AxisID.LeftTrigger.value) > this.threshold);
    }

    public Trigger rightTrigger(){
        return new Trigger(() -> this.getRawAxis(Constants.OI.AxisID.RightTrigger.value) > this.threshold);
    }

    public Trigger leftTrigger(double t){
        return new Trigger(() -> this.getRawAxis(Constants.OI.AxisID.LeftTrigger.value) > t);
    }

    public Trigger rightTrigger(double t){
        return new Trigger(() -> this.getRawAxis(Constants.OI.AxisID.RightTrigger.value) > t);
    }

    public XboxController getHID(){
        return this;
    }

    public Trigger povUp() {
        return this.pov(0);
    }

    public Trigger povUpRight() {
        return pov(45);
    }

    public Trigger povRight() {
        return pov(90);
    }

    public Trigger povDownRight() {
        return pov(135);
    }

    public Trigger povDown() {
        return pov(180);
    }

    public Trigger povDownLeft() {
        return pov(225);
    }

    public Trigger povLeft() {
        return pov(270);
    }

    public Trigger povUpLeft() {
        return pov(315);
    }

    public Trigger povCenter() {
        return pov(-1);
    }

    public Trigger pov(int angle) {
        return pov(0, angle);
    }

    public Trigger pov(int pov, int angle){
        return new Trigger(() -> this.getPOV(pov) == angle);
    }
}
