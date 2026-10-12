package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Whether a camera still sits where its configuration says, judged from where its tags land.
 *
 * <p>Each tag the camera sees, placed on the field through the estimated pose and the camera's
 * configured mount, should land on the tag. Noise, and the estimate's own small errors, put tags
 * off every which way, and average out. A camera bumped since it was calibrated puts them off the
 * same way every time, in the robot's frame, where its mount moved.
 *
 * <p>So the check averages each tag's miss in the robot's frame, over the camera's latest tags,
 * each counted alike. That average is how far off the camera's mount is. A camera knocked partway
 * through shows it in proportion to the share of its latest tags seen since, and fully once they
 * all are. Beyond a tolerance, the camera is out of calibration.
 *
 * <p>Only the level part of the miss moves the camera's measurements, since an image fixes where
 * the robot stands on the floor. That part, beyond the tolerance, is how much less the camera is
 * trusted ({@link #doubtMeters}). A camera knocked straight down still places the robot right on
 * level ground. It is out of calibration all the same, and every tag it sees lands high.
 *
 * <p>The check measures against the estimate, so it needs other cameras to hold the estimate. With
 * one camera, the estimate follows that camera, and its tags land where they should whatever its
 * mount. And a heading error in the estimate (after a gyro's scale error over many turns, say) puts
 * every camera's tags off as if each were turned on its mount. The cameras correct the heading, and
 * the offsets fade with it.
 */
public final class MountCheck {
  // The latest misses, each along the robot's x, y and z, and how far its tag was from the robot.
  // They are kept in a ring: arrays of a fixed length, where the newest overwrites the oldest once
  // the ring is full.
  private final double[] missX;
  private final double[] missY;
  private final double[] missZ;
  private final double[] tagDistance;

  /** The slot the next sighting goes into. */
  private int next;

  /** How many sightings are kept so far, which is the ring's length once it is full. */
  private int count;

  // The sums of what the ring holds, kept up as sightings come and go, so that an average costs
  // nothing to ask for.
  private double sumX;
  private double sumY;
  private double sumZ;
  private double sumDistance;

  /**
   * @param sightings how many of the latest tags the camera is judged by. It should be enough that
   *     noise and the estimate's errors average out, and few enough that a bump shows within
   *     seconds.
   */
  public MountCheck(int sightings) {
    if (sightings < 1) {
      throw new IllegalArgumentException("judge by at least one sighting: " + sightings);
    }
    missX = new double[sightings];
    missY = new double[sightings];
    missZ = new double[sightings];
    tagDistance = new double[sightings];
  }

  /**
   * A tag the camera saw.
   *
   * @param robot where the robot was when the image was taken, raised and tilted as it was
   * @param seen the tag, placed through that pose and the camera's configured mount
   * @param real where the tag really is
   */
  public void add(Pose3d robot, Pose3d seen, Pose3d real) {
    // The miss on the field, turned into the robot's frame by undoing the robot's rotation.
    Translation3d miss =
        seen.getTranslation().minus(real.getTranslation()).rotateBy(robot.getRotation().inverse());
    int slot = next;
    next = (next + 1) % missX.length;
    boolean full = count == missX.length;
    if (full) {
      // The oldest sighting is in the slot, and it leaves the sums as the newest takes its place.
      sumX -= missX[slot];
      sumY -= missY[slot];
      sumZ -= missZ[slot];
      sumDistance -= tagDistance[slot];
    } else {
      count++;
    }
    missX[slot] = miss.getX();
    missY[slot] = miss.getY();
    missZ[slot] = miss.getZ();
    tagDistance[slot] = seen.getTranslation().getDistance(robot.getTranslation());
    sumX += missX[slot];
    sumY += missY[slot];
    sumZ += missZ[slot];
    sumDistance += tagDistance[slot];
    // Once around the ring, at its last slot, the sums are added up afresh. What is taken off a
    // sum never quite cancels what was put on, and over a match the difference would add up.
    if (slot == missX.length - 1) {
      sumX = sum(missX);
      sumY = sum(missY);
      sumZ = sum(missZ);
      sumDistance = sum(tagDistance);
    }
  }

  private static double sum(double[] values) {
    double sum = 0;
    for (double value : values) {
      sum += value;
    }
    return sum;
  }

  /** Whether the camera has seen enough tags to judge its mount by. */
  public boolean judged() {
    return count == missX.length;
  }

  /**
   * How far off, on average, the camera's latest tags land. A camera mounted 4 in lower than
   * configured puts every tag 4 in high. It is the length of the average miss:
   *
   * <pre>offset = |sum of the misses| / how many</pre>
   *
   * @return NaN until the camera has seen enough tags to judge
   */
  public double offsetMeters() {
    if (!judged()) {
      return Double.NaN;
    }
    return Math.sqrt(sumX * sumX + sumY * sumY + sumZ * sumZ) / count;
  }

  /**
   * How far, on average, the camera's latest tags were from the robot. A mount turned by an angle
   * puts them that angle times this off.
   *
   * @return NaN until the camera has seen enough tags to judge
   */
  public double distanceMeters() {
    if (!judged()) {
      return Double.NaN;
    }
    return sumDistance / count;
  }

  /** Whether the camera's tags land farther off than a calibrated camera's do. */
  public boolean outOfCalibration(double toleranceMeters) {
    return offsetMeters() > toleranceMeters;
  }

  /**
   * How far the camera's measurements may be off from where its mount moved, in meters. It is none
   * while its tags land within the tolerance, and none until the mount is judged.
   *
   * <p>The tolerance stands for what a calibrated camera shows, which is mostly the estimate's own
   * error and not the camera's. So it comes off the level offset the way independent errors add, in
   * squares:
   *
   * <pre>
   * level offset² = (sum of x misses)² + (sum of y misses)², over (how many)²
   * doubt         = sqrt(level offset² - tolerance²)
   * </pre>
   */
  public double doubtMeters(double toleranceMeters) {
    if (!judged()) {
      return 0;
    }
    double levelOffsetSquared = (sumX * sumX + sumY * sumY) / ((double) count * count);
    double toleranceSquared = toleranceMeters * toleranceMeters;
    if (levelOffsetSquared <= toleranceSquared) {
      return 0;
    }
    return Math.sqrt(levelOffsetSquared - toleranceSquared);
  }
}
