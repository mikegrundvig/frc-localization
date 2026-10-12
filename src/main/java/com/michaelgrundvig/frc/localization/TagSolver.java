package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The geometry that turns a camera's view of AprilTags into where the robot is. Field and robot
 * frames as written in {@code com.michaelgrundvig.frc.world.field}; a camera's frame has x out of
 * the lens, y left, and z up, as WPILib and PhotonVision give it.
 */
public final class TagSolver {
  private TagSolver() {}

  /** The robot's pose, from its camera's pose on the field. */
  public static Pose3d robotFromCamera(Pose3d fieldToCamera, Transform3d robotToCamera) {
    return fieldToCamera.transformBy(robotToCamera.inverse());
  }

  /** The robot's pose, from a tag's known pose and where the camera saw it. */
  public static Pose3d robotFromTag(
      Pose3d fieldToTag, Transform3d cameraToTag, Transform3d robotToCamera) {
    return robotFromCamera(fieldToTag.transformBy(cameraToTag.inverse()), robotToCamera);
  }

  /**
   * Where the robot stands, from where one tag is, given how the robot is turned and tilted. A
   * single tag's orientation can be ambiguous; where it is, is not. The heading comes from the
   * gyro, so the tag only has to fix the position, and the tag's known height is a free check on
   * the view.
   *
   * @param robotToTag the tag as seen from the robot: the camera's mount, then the camera's view
   * @param attitude the robot's heading on the field, with its pitch and roll
   */
  public static PositionFix positionFromTag(
      Pose3d fieldToTag, Transform3d robotToTag, Rotation3d attitude) {
    Translation3d offset = robotToTag.getTranslation().rotateBy(attitude);
    return new PositionFix(
        new Translation2d(fieldToTag.getX() - offset.getX(), fieldToTag.getY() - offset.getY()),
        fieldToTag.getZ() - offset.getZ());
  }

  /**
   * A position from one tag.
   *
   * @param position the robot's center, on the floor
   * @param heightErrorMeters how far the tag's known height misses the height the view puts it at:
   *     a bad view, a bad camera mount, or a tilted robot
   */
  public record PositionFix(Translation2d position, double heightErrorMeters) {}
}
