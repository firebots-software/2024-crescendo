package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

// Constructs a Pose2d array of the note locations by a specific indexing so they can be accessed
// by the eventual autonomous chooser
public enum NoteLocation {
  AMPSIDE(Constants.Landmarks.AMPSIDE_NOTE_LOCATION),
  MIDDLE(Constants.Landmarks.MIDDLE_NOTE_LOCATION),
  STAGESIDE(Constants.Landmarks.STAGESIDE_NOTE_LOCATION),
  MIDLINE_FROM_AMP1(Constants.Landmarks.MIDLINE_FROM_AMP1_NOTE_LOCATION),
  MIDLINE_FROM_AMP2(Constants.Landmarks.MIDLINE_FROM_AMP2_NOTE_LOCATION),
  MIDLINE_FROM_AMP3(Constants.Landmarks.MIDLINE_FROM_AMP3_NOTE_LOCATION),
  MIDLINE_FROM_AMP4(Constants.Landmarks.MIDLINE_FROM_AMP4_NOTE_LOCATION),
  MIDLINE_FROM_AMP5(Constants.Landmarks.MIDLINE_FROM_AMP5_NOTE_LOCATION);

  private final Pose2d pose;

  private NoteLocation(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getNoteLocation() {
    return this.pose;
  }
}
