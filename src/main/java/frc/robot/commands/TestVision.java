package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;

public class TestVision extends Command{
    private PhotonVision photonVision;

    public TestVision(PhotonVision photonVision) {
        this.photonVision = photonVision;
        addRequirements(photonVision);
    }

    @Override
    public void initialize() {
        // what does the command immediately do when first told to run?
        
    }
    
    @Override
    public void execute() {
        // what does the command do every ~1/50th of a second while running?
    }

    @Override
    public boolean isFinished() {
        // when does the command stop itself?
        // does it run only initialize and exit? does it never stop by itself?
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // what does the command do when told to stop (either by itself or externally)?
    }
}
