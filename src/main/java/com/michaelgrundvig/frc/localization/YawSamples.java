package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.config.Checks;
import java.util.Arrays;
import org.jspecify.annotations.Nullable;

/**
 * An IMU's yaw readings since the last loop, oldest first: when each was taken, and the yaw then.
 * On a robot they are taken faster than the loop runs, so that odometry turns with every one.
 *
 * <p>The readings are two plain arrays and a count, read one reading at a time. A drivetrain hands
 * these over every loop, and lists of boxed numbers would make a dozen objects each time.
 *
 * <p>Nothing here changes once it is made. Readings made with {@link #of} are copies. Readings made
 * with {@link #handedOver} are the caller's own arrays, which is what makes them free: see there
 * for what the caller promises.
 */
public final class YawSamples {
  /** No readings: what an IMU that is not there has. */
  public static final YawSamples NONE = new YawSamples(new long[0], new double[0], 0);

  private final long[] timesNanos;
  private final double[] yawsRadians;

  /** How many readings there are: the arrays may be longer, and only this many count. */
  private final int count;

  private YawSamples(long[] timesNanos, double[] yawsRadians, int count) {
    this.timesNanos = timesNanos;
    this.yawsRadians = yawsRadians;
    this.count = count;
  }

  /**
   * Readings copied from two arrays, which the caller keeps and may change afterwards.
   *
   * @param timesNanos when each reading was taken, oldest first
   * @param yawsRadians the yaw at each of those times
   */
  public static YawSamples of(long[] timesNanos, double[] yawsRadians) {
    int times = timesNanos.length;
    int readings = yawsRadians.length;
    Checks.on(ImuReadings.class)
        .check(
            "yawsRadians",
            () ->
                readings == times
                    ? null
                    : "must have one reading for each time: "
                        + times
                        + " times, "
                        + readings
                        + " readings")
        .orThrow();
    return new YawSamples(timesNanos.clone(), yawsRadians.clone(), times);
  }

  /**
   * Readings that are the caller's own arrays, not copies, so that handing them over makes no new
   * arrays. The caller promises never to write to either array again. A drivetrain whose hardware
   * layer makes new arrays for each loop's readings can promise that.
   *
   * @param timesNanos when each reading was taken, oldest first
   * @param yawsRadians the yaw at each of those times
   * @param count how many readings count, from the start of both arrays
   */
  public static YawSamples handedOver(long[] timesNanos, double[] yawsRadians, int count) {
    if (count < 0 || count > timesNanos.length || count > yawsRadians.length) {
      throw new IllegalArgumentException(
          count
              + " readings counted, of "
              + timesNanos.length
              + " times and "
              + yawsRadians.length
              + " yaws");
    }
    return new YawSamples(timesNanos, yawsRadians, count);
  }

  /** How many readings there are. */
  public int count() {
    return count;
  }

  /** When a reading was taken, in nanoseconds on the robot's clock. */
  public long timeNanos(int reading) {
    return timesNanos[inRange(reading)];
  }

  /** The yaw at a reading, in radians. */
  public double yawRadians(int reading) {
    return yawsRadians[inRange(reading)];
  }

  /** Whether every reading's yaw is a number. */
  public boolean areAllNumbers() {
    for (int reading = 0; reading < count; reading++) {
      if (!Double.isFinite(yawsRadians[reading])) {
        return false;
      }
    }
    return true;
  }

  /** A reading's place in the arrays, which may be longer than the readings that count. */
  private int inRange(int reading) {
    if (reading < 0 || reading >= count) {
      throw new IndexOutOfBoundsException("reading " + reading + " of " + count);
    }
    return reading;
  }

  @Override
  public boolean equals(@Nullable Object other) {
    return other instanceof YawSamples samples
        && count == samples.count
        && Arrays.equals(timesNanos, 0, count, samples.timesNanos, 0, count)
        && Arrays.equals(yawsRadians, 0, count, samples.yawsRadians, 0, count);
  }

  @Override
  public int hashCode() {
    int hash = count;
    for (int reading = 0; reading < count; reading++) {
      hash = 31 * (31 * hash + Long.hashCode(timesNanos[reading]));
      hash += Double.hashCode(yawsRadians[reading]);
    }
    return hash;
  }

  @Override
  public String toString() {
    StringBuilder shown = new StringBuilder("YawSamples[");
    for (int reading = 0; reading < count; reading++) {
      if (reading > 0) {
        shown.append(", ");
      }
      shown.append(yawsRadians[reading]).append(" rad at ").append(timesNanos[reading]);
      shown.append(" ns");
    }
    return shown.append(']').toString();
  }
}
