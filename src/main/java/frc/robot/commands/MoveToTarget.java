package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTarget extends Command {
    private Pose2d targetLocation;
    private SwerveSubsystem swerve;

    public MoveToTarget(SwerveSubsystem swerve, Pose2d targetLocation) {
        this.swerve = SwerveSubsystem.getInstance();
        this.targetLocation = targetLocation;
        // Adds the subsystem as a requirement (prevents two commands from acting on
        // subsystem at once)
        addRequirements(swerve);
    }
    // Create a list of bezier points from poses. Each pose represents one waypoint.
    // // The rotation component of the pose should be the direction of travel. Do
    // not use holonomic rotation.
    // bezierPoints = PathPlannerPath.bezierFromPoses(
    // new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
    // new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
    // new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    // );List<Translation2d>

    // // Create the path using the bezier points created above
    // PathPlannerPath path = new PathPlannerPath(
    // bezierPoints,
    // new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints
    // for this path. If using a differential drivetrain, the angular constraints
    // have no effect.
    // new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can
    // set a holonomic rotation here. If using a differential drivetrain, the
    // rotation will have no effect.
    // );

    // // Prevent the path from being flipped if the coordinates are already correct
    // path.preventFlipping =true;

    private Command pathFindingCommand;

    @Override
    public void initialize() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        this.pathFindingCommand = AutoBuilder.pathfindToPose(
                this.targetLocation,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );
    }

    public void execute() {
        pathFindingCommand.execute(); 
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (pathFindingCommand != null)
            pathFindingCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pathFindingCommand == null ? false : pathFindingCommand.isFinished();
    }
}