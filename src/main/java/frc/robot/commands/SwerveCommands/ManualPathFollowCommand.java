package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.function.DoubleSupplier;

public class ManualPathFollowCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PathPlannerPath path;
    private final DoubleSupplier speedSupplier;
    private int currentPointIndex = 0;
    private static final double MAX_SPEED = 0.4;
    private final List<PathPoint> pathPoints;
    
    public ManualPathFollowCommand(SwerveSubsystem swerve, PathPlannerPath path, DoubleSupplier speedSupplier) {
        this.swerve = swerve;
        this.path = path;
        this.speedSupplier = speedSupplier;
        this.pathPoints = path.getAllPathPoints();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        
        double scaledSpeed = speed * MAX_SPEED;
        
        if (scaledSpeed > 0 && currentPointIndex < pathPoints.size() - 1) {
            Translation2d currentPos = swerve.getPose().getTranslation();
            Translation2d nextPoint = pathPoints.get(currentPointIndex + 1).position;
            if (currentPos.getDistance(nextPoint) < 0.1) {
                currentPointIndex++;
            }
        } else if (scaledSpeed < 0 && currentPointIndex > 0) {
            Translation2d currentPos = swerve.getPose().getTranslation();
            Translation2d prevPoint = pathPoints.get(currentPointIndex - 1).position;
            if (currentPos.getDistance(prevPoint) < 0.1) {
                currentPointIndex--;
            }
        }

        PathPoint targetPoint = pathPoints.get(currentPointIndex);
        
        Translation2d currentPos = swerve.getPose().getTranslation();
        Translation2d targetPos = targetPoint.position;
        Rotation2d heading = targetPos.minus(currentPos).getAngle();
        
        if (scaledSpeed < 0) {
            heading = heading.plus(new Rotation2d(Math.PI));
        }
        
        ChassisSpeeds speeds = new ChassisSpeeds(
            scaledSpeed * heading.getCos(),
            scaledSpeed * heading.getSin(),
            targetPoint.rotationTarget.getTarget().minus(swerve.getPose().getRotation()).getRadians() * 2
        );
        
        swerve.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static Command createManualPathCommand(SwerveSubsystem swerve, DoubleSupplier speedSupplier) {
        Pose2d currentPose = swerve.getPose();
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            currentPose,
            new Pose2d(currentPose.getX(), currentPose.getY() + 1.0, currentPose.getRotation())
        );
        
        PathConstraints constraints = new PathConstraints(
            MAX_SPEED,
            MAX_SPEED,
            2 * Math.PI,
            4 * Math.PI
        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            new GoalEndState(0.0, currentPose.getRotation())
        );

        path.preventFlipping = true;

        return new ManualPathFollowCommand(swerve, path, speedSupplier);
    }
}