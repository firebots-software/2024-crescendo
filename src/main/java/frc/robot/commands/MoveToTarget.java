package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;


public class MoveToTarget extends Command {
    private ArrayList<Pose2d> pathPoints = new ArrayList<Pose2d>();
    private SwerveSubsystem swerve;

    public MoveToTarget(SwerveSubsystem swerve, ArrayList<Pose2d> pathPoints) {
        this.swerve = SwerveSubsystem.getInstance();

        // Adds the subsystem as a requirement (prevents two commands from acting on
        // subsystem at once)
        addRequirements(swerve);
    }
    // Create a list of bezier points from poses. Each pose represents one waypoint.
    // // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    //  bezierPoints = PathPlannerPath.bezierFromPoses(
    //         new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
    //         new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
    //         new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    // );List<Translation2d>

    // // Create the path using the bezier points created above
    // PathPlannerPath path = new PathPlannerPath(
    //         bezierPoints,
    //         new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    //         new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    // );

    // // Prevent the path from being flipped if the coordinates are already correct
    // path.preventFlipping =true;
    @Override
    public void initialize() {

    }
}