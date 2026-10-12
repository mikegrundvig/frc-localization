package com.michaelgrundvig.frc.localization;

import org.wpilib.math.util.MathUtil;

/**
 * A gyro's recent yaw readings, for its yaw at the moment of each wheel reading. A gyro that is
 * read less often than the wheels (an IMU read once a loop, while the wheels are read at 250 Hz)
 * still turns every odometry step this way.
 *
 * <p>Between two readings the yaw is interpolated the short way around. At a reading's own time it
 * is that reading exactly. Past the newest reading it holds the newest.
 *
 * <p>{@link #at} runs for every wheel reading, 250 times a second, so it works on plain arrays and
 * numbers and makes nothing.
 */
public final class YawTrack {
  /** This loop's readings, oldest first: when each was taken, and the yaw then, in radians. */
  private long[] timesNanos = new long[0];

  private double[] yawsRadians = new double[0];

  /**
   * The newest reading of the loop before, which a moment before this loop's first reading is
   * interpolated from. The time and the yaw mean something only while there is one.
   */
  private boolean hasPrevious;

  private long previousTimeNanos;
  private double previousYawRadians;

  /**
   * This loop's readings, oldest first. The previous loop's newest reading is kept, so that moments
   * before this loop's first reading interpolate from it.
   */
  public void update(long[] timestampsNanos, double[] yawsRadians) {
    if (timestampsNanos.length != yawsRadians.length) {
      throw new IllegalArgumentException("one time for each yaw");
    }
    boolean hadReadings = timesNanos.length > 0;
    if (hadReadings) {
      hasPrevious = true;
      previousTimeNanos = timesNanos[timesNanos.length - 1];
      previousYawRadians = this.yawsRadians[this.yawsRadians.length - 1];
    }
    timesNanos = timestampsNanos.clone();
    this.yawsRadians = yawsRadians.clone();
  }

  /** Forgets every reading. The gyro was lost, so its old readings say nothing about now. */
  public void clear() {
    timesNanos = new long[0];
    yawsRadians = new double[0];
    hasPrevious = false;
  }

  /** Whether there's a reading this loop. */
  public boolean hasReadings() {
    return timesNanos.length > 0;
  }

  /** The yaw at a moment, in radians. Call only when there's a reading this loop. */
  public double at(long timestampNanos) {
    if (timesNanos.length == 0) {
      throw new IllegalStateException("no readings this loop");
    }
    // Walk forward through the readings to the first one taken after the moment. The reading
    // before it, which may be the loop before's newest, is the one to interpolate from.
    boolean hasBefore = hasPrevious;
    long beforeTimeNanos = previousTimeNanos;
    double beforeYawRadians = previousYawRadians;
    for (int k = 0; k < timesNanos.length; k++) {
      if (timesNanos[k] == timestampNanos) {
        return yawsRadians[k];
      }
      if (timesNanos[k] > timestampNanos) {
        // With no reading before the moment, the earliest reading stands. So it does when the
        // reading before isn't earlier than this one, which leaves no span to interpolate over.
        boolean spanToInterpolateOver = hasBefore && beforeTimeNanos < timesNanos[k];
        if (!spanToInterpolateOver) {
          return yawsRadians[k];
        }
        return between(
            beforeTimeNanos, beforeYawRadians, timesNanos[k], yawsRadians[k], timestampNanos);
      }
      hasBefore = true;
      beforeTimeNanos = timesNanos[k];
      beforeYawRadians = yawsRadians[k];
    }
    return yawsRadians[yawsRadians.length - 1];
  }

  /**
   * The yaw at a moment between two readings. The later yaw is taken as the short way around from
   * the earlier one, so a yaw that wraps from just under a half turn to just over minus a half turn
   * is a small step, not nearly a whole turn back:
   *
   * <pre>
   * fraction = (moment - earlier time) / (later time - earlier time)
   * yaw      = earlier yaw + fraction × (later yaw - earlier yaw, wrapped to within half a turn)
   * </pre>
   */
  private static double between(
      long earlierNanos,
      double earlierYawRadians,
      long laterNanos,
      double laterYawRadians,
      long timestampNanos) {
    double fraction = (double) (timestampNanos - earlierNanos) / (laterNanos - earlierNanos);
    return earlierYawRadians
        + fraction * MathUtil.angleModulus(laterYawRadians - earlierYawRadians);
  }
}
