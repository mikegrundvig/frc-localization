package com.michaelgrundvig.frc.localization;

/**
 * What the robot's other sensors and its state say about trusting its wheels right now. Give it to
 * the estimator once per loop, before that loop's odometry. The estimator weighs each odometry step
 * by the conditions it was taken under and keeps that weight with the step, so a replay of the
 * history weighs every step as it was weighed then.
 *
 * @param tiltRadians how far the robot is tilted from level, from the IMU. Tilted beyond what the
 *     ground explains, its wheels may have lost contact, and their travel counts for less.
 * @param groundSlopeRadians how much of that tilt the ground under the robot explains: on a known
 *     bump's ramp the wheels still roll on the ground, and their travel counts in full, at the
 *     slope's cosine across the field. Zero on flat ground, or where the terrain isn't known.
 * @param collisionShare how much of a collision the IMU just felt, from none to a whole one ({@link
 *     SlipCheck#collisionShare}): the robot may have been moved without its wheels seeing it. A
 *     whole one is a jolt beyond anything driving explains, and the robot counts as just hit; a
 *     jolt the wheels partly explain is the share they don't, and adds that share of a collision's
 *     spread. Outside zero to one, or not a number, it's taken as the nearest of them, or none.
 * @param slipMetersPerSecond how fast the wheels may be sliding over the floor together, as the IMU
 *     sees them ({@link SlipCheck}): the gyro turning the robot less or more than they do, the
 *     accelerometer feeling it speed up less or more. Zero when the IMU agrees with them, or there
 *     is none. Slip that sets the wheels against each other shows in their residual instead ({@link
 *     BodyMotion#residualMeters}). The travel it may have added counts as suspect. Negative, not a
 *     number or infinite, it's taken as none: it's built inside the robot's loop, which mustn't
 *     stop for it, and an infinite doubt would leave the estimate nothing to stand on.
 * @param disabled whether the robot is disabled, when people pick it up and carry it where its
 *     wheels can't see: the position's uncertainty grows with time, so the cameras lead until it's
 *     enabled (a top team trusted vision loosely while disabled, tightly once enabled)
 */
public record DriveConditions(
    double tiltRadians,
    double groundSlopeRadians,
    double collisionShare,
    double slipMetersPerSecond,
    boolean disabled) {
  /** Level, nothing hit, wheels gripping, enabled. */
  public static final DriveConditions NORMAL = new DriveConditions(0, 0, 0, 0, false);

  public DriveConditions {
    if (!(tiltRadians >= 0) || Double.isInfinite(tiltRadians)) {
      throw new IllegalArgumentException("tilt is an angle from level: " + tiltRadians);
    }
    if (!(groundSlopeRadians >= 0) || !(groundSlopeRadians <= tiltRadians)) {
      throw new IllegalArgumentException(
          "the ground explains some or all of the tilt, never more: " + groundSlopeRadians);
    }
    collisionShare = asAShare(collisionShare);
    slipMetersPerSecond = asASpeedOfSlip(slipMetersPerSecond);
  }

  /** A share from none to a whole one. Below zero, or not a number, is none; above one is one. */
  private static double asAShare(double share) {
    boolean any = share > 0;
    if (!any) {
      return 0;
    }
    return Math.min(share, 1);
  }

  /** A speed of slip, which is none when it is negative, not a number or infinite. */
  private static double asASpeedOfSlip(double metersPerSecond) {
    boolean usable = metersPerSecond > 0 && Double.isFinite(metersPerSecond);
    if (!usable) {
      return 0;
    }
    return metersPerSecond;
  }

  /** Whether the IMU just felt a whole collision: a jolt beyond anything driving explains. */
  public boolean collision() {
    return collisionShare >= 1;
  }

  /** How far the robot tilts beyond what the ground explains. */
  public double unexplainedTiltRadians() {
    return tiltRadians - groundSlopeRadians;
  }
}
