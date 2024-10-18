package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToNote extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem camera;
    private Supplier<Double> frontBackFunction;
    private Supplier<Double> leftRightFunction;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable table = inst.getTable("photonvision/ObjDetectionCam");
    
    public TurnToNote(SwerveSubsystem swerve, VisionSubsystem camera, Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction) {
        this.swerve = swerve;
        this.camera = camera;
        this.frontBackFunction = frontBackFunction;
        this.leftRightFunction = leftRightFunction;
        
        // addRequirements(swerve, camera);
    }
    
    @Override
    public void execute() {
        double yawAngle = table.getEntry("targetYaw").getDouble(0.0);
        boolean noteFound = table.getEntry("noteFound").getBoolean(false); //TODO: check the actual entry name, otherwise wont work
        
        if (noteFound) {
            
            Rotation2d currentRotation = swerve.getState().Pose.getRotation();
            

            Rotation2d targetRotation = currentRotation.plus(
                Rotation2d.fromDegrees(yawAngle)
            );
            
            // Piggybacking off of existing SwerveTurnToAngle
            new SwerveLockedAngleCmd(
                () -> frontBackFunction.get(),
                () -> leftRightFunction.get(),
                () -> targetRotation,
                () -> 1.0,
                swerve
            ).schedule();
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}