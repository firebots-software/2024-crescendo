package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xPos;
    private Supplier<Double> yPos;
    
    public SwerveCommand(Supplier<Double> xPos, Supplier<Double> yPos){
        swerveSubsystem = SwerveSubsystem.getInstance();
        this.xPos=xPos;
        this.yPos=yPos;
    }

    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x_JoyStick = xPos.get();
        double y_JoyStick = yPos.get();
        
        double velocityLength = Math.min(Math.sqrt(x_JoyStick*x_JoyStick+y_JoyStick*y_JoyStick),1.0);
        //normalization
        x_JoyStick = x_JoyStick/velocityLength;
        y_JoyStick = y_JoyStick/velocityLength;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_JoyStick, y_JoyStick, 0.1, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }   
}
