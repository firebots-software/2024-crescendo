package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCmd extends InstantCommand {
    public ZeroHeadingCmd(SwerveSubsystem swerveSubsystem) {
        super(() -> {
            swerveSubsystem.zeroHeading();
            swerveSubsystem.zeroPitch();
        }, swerveSubsystem);
    }
}