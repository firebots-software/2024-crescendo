package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToTarget extends Command {
    private Pose2d targetLocation;
    private SwerveSubsystem swerve;
    private Command pathFindingCommand;

    public MoveToTarget(SwerveSubsystem swerve, Pose2d targetLocation) {
        this.swerve = SwerveSubsystem.getInstance();
        this.targetLocation = targetLocation;
        addRequirements(swerve);
    }

    

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

    // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // // Add a button to run pathfinding commands to SmartDashboard
    // AutoBuilder.pathfindToPose(
    //   new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   2.0
    // );
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   0
    // ));

    // // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // // This example will simply move the robot 2m in the +X field direction
    // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    //   Pose2d currentPose = swerve.getPose();
      
    //   // The rotation component in these poses represents the direction of travel
    //   Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    //   Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

    //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    //   PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints, 
    //     new PathConstraints(
    //       4.0, 4.0, 
    //       Units.degreesToRadians(360), Units.degreesToRadians(540)
    //     ),  
    //     new GoalEndState(0.0, currentPose.getRotation())
    //   );

    //   // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //   path.preventFlipping = true;

    //   AutoBuilder.followPath(path).schedule();
    // }));
//   }
}