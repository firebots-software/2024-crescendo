package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToNote extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem camera;
    private final NetworkTable noteTable;
    private final NetworkTableEntry noteAngleEntry;
    private final NetworkTableEntry noteFoundEntry;
    private final NetworkTableEntry noteDistanceEntry;
    private Supplier<Double> frontBackFunction;
    private Supplier<Double> leftRightFunction;
    
    public TurnToNote(SwerveSubsystem swerve, VisionSubsystem camera, Supplier<Double> frontBackFunction, Supplier<Double> leftRightFunction) {
        this.swerve = swerve;
        this.camera = camera;
        this.frontBackFunction = frontBackFunction;
        this.leftRightFunction = leftRightFunction;

        noteTable = NetworkTableInstance.getDefault().getTable("Note");
        noteAngleEntry = noteTable.getEntry("targetAngle");
        noteFoundEntry = noteTable.getEntry("noteFound");
        noteDistanceEntry = noteTable.getEntry("targetDistance");
        
        addRequirements(swerve, camera);
    }
    
    @Override
    public void execute() {
        if (noteFoundEntry.getBoolean(false)) {
            
            Rotation2d currentRotation = swerve.getState().Pose.getRotation();
            

            Rotation2d targetRotation = currentRotation.plus(
                Rotation2d.fromDegrees(noteAngleEntry.getDouble(0))
            );
            
            // Piggybacking off of existing SwerveTurnToAngle
            new SwerveTurnToAngle(
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