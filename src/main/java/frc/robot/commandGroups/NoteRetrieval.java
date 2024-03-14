package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.MoveToTarget;
import frc.robot.commands.Auton.PathfindToTarget;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MiscUtils;
import frc.robot.util.NoteLocation;
import java.util.function.Supplier;

public class NoteRetrieval extends SequentialCommandGroup {
  public NoteRetrieval(SwerveSubsystem swerve, NoteLocation targetNote, Supplier<Boolean> redside) {
    if (MiscUtils.isCloseNote(targetNote)) {
      addCommands(
          MoveToTarget.withMirror(
              swerve,
              redside,
              targetNote.getNoteLocation().getRotation(),
              (redside.get())
                  ? MiscUtils.reflectAcrossMidline(
                      new Pose2d(
                          swerve.getState().Pose.getTranslation(),
                          new Rotation2d(
                              -1,
                              swerve.getState().Pose.getY() - targetNote.getNoteLocation().getY())))
                  : new Pose2d(
                      swerve.getState().Pose.getTranslation(),
                      new Rotation2d(
                          -1, swerve.getState().Pose.getY() - targetNote.getNoteLocation().getY())),
              (redside.get())
                  ? MiscUtils.reflectAcrossMidline(
                      new Pose2d(
                          new Translation2d(
                              targetNote.getNoteLocation().getX()
                                  - (Units.inchesToMeters(18)
                                      * Math.cos(
                                          targetNote.getNoteLocation().getRotation().getRadians())),
                              targetNote.getNoteLocation().getY()
                                  - (Units.inchesToMeters(18)
                                      * Math.sin(
                                          targetNote
                                              .getNoteLocation()
                                              .getRotation()
                                              .getRadians()))),
                          targetNote.getNoteLocation().getRotation()))
                  : new Pose2d(
                      new Translation2d(
                          targetNote.getNoteLocation().getX()
                              - (Units.inchesToMeters(18)
                                  * Math.cos(
                                      targetNote.getNoteLocation().getRotation().getRadians())),
                          targetNote.getNoteLocation().getY()
                              - (Units.inchesToMeters(18)
                                  * Math.sin(
                                      targetNote.getNoteLocation().getRotation().getRadians()))),
                      targetNote.getNoteLocation().getRotation())));
    } else {
      addCommands(
          PathfindToTarget.withMirror(
              swerve,
              redside,
              new Pose2d(
                  new Translation2d(
                      targetNote.getNoteLocation().getX() - 1, targetNote.getNoteLocation().getY()),
                  targetNote.getNoteLocation().getRotation())),
          MoveToTarget.withMirror(
              swerve,
              redside,
              ((redside.get()) ? new Rotation2d(Math.PI) : new Rotation2d()),
              new Pose2d(
                  new Translation2d(
                      targetNote.getNoteLocation().getX() - 1, targetNote.getNoteLocation().getY()),
                  targetNote.getNoteLocation().getRotation()),
              new Pose2d(
                  new Translation2d(
                      targetNote.getNoteLocation().getX()
                          + ((redside.get())
                              ? Units.inchesToMeters(18)
                              : Units.inchesToMeters(-18)),
                      targetNote.getNoteLocation().getY()),
                  new Rotation2d())));
    }
  }
}
