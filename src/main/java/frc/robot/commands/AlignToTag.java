package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToTag extends Command {
    private PhotonVision photonVision;
    private SwerveSubsystem swerveSubsystem;
    private int tagID;

    public AlignToTag(PhotonVision photonVision, SwerveSubsystem swerveSubsystem, int tagID) {
        this.photonVision = photonVision;
        this.swerveSubsystem = swerveSubsystem;
        this.tagID = tagID;
        addRequirements(photonVision);
    }

    @Override
    public void initialize() {
        // what does the command immediately do when first told to run?
        
    }
    
    @Override
    public void execute() {
        // If detected target doesn't match desired tag, stop and return.
        // Target variable automatically updated when matchesTagID() is called.
        if(!photonVision.matchesTagID(this.tagID)) {
            final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
            swerveSubsystem.setControl(brake);
            return;
        };

        // Get the camera's offset from the AprilTag as a Transform3d
        Transform3d camToTag = photonVision.getBestCamToTarget(false);

        // Forward velocity in m/s
        double x = 0;

        // Velocity to the left in m/s
        double maxV = 1;
        double y = clamp(camToTag.getY(), -maxV, maxV);

        // Turn rate counterclockwise in radians/s
        double maxT = Math.PI/4;
        double minT = Math.PI/8;
        double turn = clamp(withLowBound(camToTag.getRotation().getZ(), minT), -maxT, maxT);

        // Drive robot using SwerveSubsystem
        final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        swerveSubsystem.setControl(drive
        .withVelocityX(x)
        .withVelocityY(y)
        .withRotationalRate(turn));
    }

    @Override
    public boolean isFinished() {
        // when does the command stop itself?
        // does it run only initialize and exit? does it never stop by itself?
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        swerveSubsystem.setControl(brake);
    }

    /**
     * Returns n, clamped between min and max.
     * @param n The number to clamp
     * @param min The minimum bound
     * @param max The maximum bound
     * @return Clamped version of n
     */
    private static double clamp(double n, double min, double max) {
        return Math.max(min, Math.min(max, n));
    }

    /**
     * Returns n, but with a lower bound applied. The sign of n is retained.
     * @apiNote The lower bound is applied to the absolute value of n, and then the original sign is added back in.
     * @apiNote withLowBound(0.25, 0.5) -> 0.5
     * @apiNote withLowBound(-0.1, 0.4) -> -0.4
     * @apiNote withLowBound(1.8, 0.5) -> 1.8
     * @param n The number to apply lower bound to
     * @param min The minimum boundary
     * @return The number with the applied lower bound
     */
    private static double withLowBound(double n, double min) {
        return n/Math.abs(n) * (Math.max(min, Math.abs(n)));
    }
}
