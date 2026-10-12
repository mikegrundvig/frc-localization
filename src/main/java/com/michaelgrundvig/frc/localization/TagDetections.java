package com.michaelgrundvig.frc.localization;

import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The AprilTags one camera saw in one image, as raw as the vision software gives them. Keeping the
 * raw detections, not only a solved pose, lets the robot solve them again with its own gyro
 * heading, and lets a replay run a changed solver against a real match.
 *
 * @param timestampNanos when the image was captured
 * @param camera the camera's name, as in the robot's configuration
 * @param tags each tag seen
 * @param multiTag the vision software's solution from all the tags at once, if it made one
 */
public record TagDetections(
    long timestampNanos, String camera, List<Tag> tags, @Nullable MultiTag multiTag) {
  public TagDetections {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (camera.isBlank()) {
      throw new IllegalArgumentException("detections need their camera's name");
    }
    tags = List.copyOf(tags);
  }

  /**
   * One tag in the image.
   *
   * @param id the tag's number
   * @param corners the four corners in the image, in pixels
   * @param bestCameraToTag the tag's pose from the camera: the likelier of the two poses that one
   *     tag alone allows
   * @param altCameraToTag the other of the two
   * @param ambiguity how close the two are to equally likely: 0 is certain, 1 a coin toss
   */
  public record Tag(
      int id,
      List<Translation2d> corners,
      Transform3d bestCameraToTag,
      Transform3d altCameraToTag,
      double ambiguity) {
    public Tag {
      if (corners.size() != 4) {
        throw new IllegalArgumentException("tag " + id + " needs its 4 corners: " + corners.size());
      }
      corners = List.copyOf(corners);
      if (!(ambiguity >= 0 && ambiguity <= 1)) {
        throw new IllegalArgumentException(
            "tag " + id + ": ambiguity is from 0 to 1: " + ambiguity);
      }
    }
  }

  /**
   * The vision software's solution from every tag at once.
   *
   * @param fieldToCamera the camera's pose on the field
   * @param reprojectionErrorPixels how far, on average, the tags' corners land from where the
   *     solution puts them
   * @param tagIds the tags it used
   */
  public record MultiTag(
      Transform3d fieldToCamera, double reprojectionErrorPixels, List<Integer> tagIds) {
    public MultiTag {
      if (!(reprojectionErrorPixels >= 0) || Double.isInfinite(reprojectionErrorPixels)) {
        throw new IllegalArgumentException(
            "the reprojection error must be finite, and zero or more: " + reprojectionErrorPixels);
      }
      if (tagIds.isEmpty()) {
        throw new IllegalArgumentException("a multi-tag solution uses at least one tag");
      }
      tagIds = List.copyOf(tagIds);
    }
  }
}
