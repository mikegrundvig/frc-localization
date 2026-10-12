package com.michaelgrundvig.frc.localization.calibration;

import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * One tag a camera saw during a {@link CameraMountCalibration}, and how badly a candidate mount
 * explains it.
 *
 * <p>The vision software finds two poses for a single tag, the second being the tag turned over.
 * The sighting keeps both and uses whichever fits the mount the solver is converging on ({@link
 * #choose}).
 *
 * <p><b>Its residuals.</b> For a mount, the sighting says where the tag would be: the robot's pose,
 * then the mount, then the camera's view of the tag. How far that misses the tag's real place and
 * orientation is split into {@link #RESIDUALS} numbers, each divided by how well a camera sees that
 * part, so that all of them are comparable and a least-squares fit weighs them rightly:
 *
 * <ul>
 *   <li>three for the miss across the line of sight (its x, y and z on the field), which a camera
 *       sees to within millimeters, growing with the tag's distance;
 *   <li>one for the miss along the line of sight, which it sees far less well, growing with the
 *       distance squared, since only the tag's apparent size gives it;
 *   <li>three for the tag's orientation (a rotation vector), in between.
 * </ul>
 */
final class TagSighting {
  /** How many residuals a sighting has: three across, one along, three of orientation. */
  static final int RESIDUALS = 7;

  // Where each part sits among a sighting's residuals.
  private static final int ACROSS = 0;
  private static final int ALONG = 3;
  private static final int TURN = 4;

  // How well a camera sees a tag: across the line of sight, in meters per meter of distance; along
  // it, in meters per meter of distance squared; and the tag's orientation, in radians per meter.
  private static final double ACROSS_PER_METER = 0.002;
  private static final double ALONG_PER_METER_SQUARED = 0.004;
  private static final double TURN_PER_METER = 0.02;

  /** A tag closer than this is weighed as if it were this far, so no spread is ever zero. */
  private static final double NEAREST_DISTANCE_METERS = 0.1;

  private final Pose3d robot;
  private final Transform3d best;
  private final Optional<Transform3d> alternate;
  private final Pose3d fieldToTag;

  /** The pose in use: the best, or the alternate if that fits the mount better. */
  private Transform3d view;

  /** The line of sight to the tag, as a unit vector on the field, at the mount last chosen at. */
  private Translation3d lineOfSight = Translation3d.ZERO;

  // How well the camera sees this tag, at its distance: the spreads its misses are divided by.
  private double acrossSpreadMeters;
  private double alongSpreadMeters;
  private double turnSpreadRadians;

  /**
   * @param robot where the robot was, tilted as it was, when the image was taken
   * @param best the vision software's best pose of the tag, from the camera
   * @param alternate its other candidate, if it gave one
   * @param fieldToTag where the tag really is
   */
  TagSighting(Pose3d robot, Transform3d best, Optional<Transform3d> alternate, Pose3d fieldToTag) {
    this.robot = robot;
    this.best = best;
    this.alternate = alternate;
    this.fieldToTag = fieldToTag;
    view = best;
  }

  Pose3d robot() {
    return robot;
  }

  /**
   * Chooses, of the tag's two poses, the one that fits better at a mount, in place and orientation
   * together, and sets the line of sight and the spreads for it. The best pose is kept unless the
   * alternate fits strictly better.
   *
   * @return whether the choice changed
   */
  boolean choose(Transform3d mount) {
    Transform3d before = view;
    use(best, mount);
    if (alternate.isPresent()) {
      double bestMiss = sumOfSquaredResiduals(mount);
      use(alternate.get(), mount);
      boolean bestFitsAtLeastAsWell = sumOfSquaredResiduals(mount) >= bestMiss;
      if (bestFitsAtLeastAsWell) {
        use(best, mount);
      }
    }
    return view != before;
  }

  /** Takes a pose as the one in use, and works out its line of sight and spreads at a mount. */
  private void use(Transform3d pose, Transform3d mount) {
    view = pose;
    double distance = Math.max(view.getTranslation().getNorm(), NEAREST_DISTANCE_METERS);
    lineOfSight =
        view.getTranslation().div(distance).rotateBy(robot.transformBy(mount).getRotation());
    acrossSpreadMeters = ACROSS_PER_METER * distance;
    alongSpreadMeters = ALONG_PER_METER_SQUARED * distance * distance;
    turnSpreadRadians = TURN_PER_METER * distance;
  }

  private double sumOfSquaredResiduals(Transform3d mount) {
    double[] residuals = new double[RESIDUALS];
    residualsInto(mount, residuals, 0);
    double sum = 0;
    for (double residual : residuals) {
      sum += residual * residual;
    }
    return sum;
  }

  /**
   * Writes this sighting's {@link #RESIDUALS} residuals at a mount into an array.
   *
   * @param at where in the array its first residual goes
   */
  void residualsInto(Transform3d mount, double[] residuals, int at) {
    Pose3d seen = robot.transformBy(mount).transformBy(view);
    Translation3d miss = seen.getTranslation().minus(fieldToTag.getTranslation());
    double along = alongLineOfSight(miss);
    Translation3d across = miss.minus(lineOfSight.times(along));
    var turn = seen.getRotation().relativeTo(fieldToTag.getRotation()).toVector();
    residuals[at + ACROSS] = across.getX() / acrossSpreadMeters;
    residuals[at + ACROSS + 1] = across.getY() / acrossSpreadMeters;
    residuals[at + ACROSS + 2] = across.getZ() / acrossSpreadMeters;
    residuals[at + ALONG] = along / alongSpreadMeters;
    residuals[at + TURN] = turn.get(0) / turnSpreadRadians;
    residuals[at + TURN + 1] = turn.get(1) / turnSpreadRadians;
    residuals[at + TURN + 2] = turn.get(2) / turnSpreadRadians;
  }

  /** How far the tag lands from its place across the line of sight, at a mount, in meters. */
  double missAcrossLineOfSight(Transform3d mount) {
    Translation3d miss =
        robot
            .transformBy(mount)
            .transformBy(view)
            .getTranslation()
            .minus(fieldToTag.getTranslation());
    double along = alongLineOfSight(miss);
    return miss.minus(lineOfSight.times(along)).getNorm();
  }

  /** The part of a miss that lies along the line of sight: its dot product with it. */
  private double alongLineOfSight(Translation3d miss) {
    return miss.getX() * lineOfSight.getX()
        + miss.getY() * lineOfSight.getY()
        + miss.getZ() * lineOfSight.getZ();
  }
}
