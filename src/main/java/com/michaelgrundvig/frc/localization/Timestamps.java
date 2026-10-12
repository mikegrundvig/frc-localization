package com.michaelgrundvig.frc.localization;

/**
 * The robot's clock keeps nanoseconds (WPILib 2027); vision libraries and WPILib's {@code Timer}
 * report seconds on the same clock. Measurements carry nanoseconds, converted here and nowhere
 * else.
 */
public final class Timestamps {
  public static final long NANOS_PER_SECOND = 1_000_000_000L;

  private Timestamps() {}

  /** A time in seconds on the robot's clock, such as a camera result's, in nanoseconds. */
  public static long fromSeconds(double seconds) {
    if (!Double.isFinite(seconds) || seconds < 0) {
      throw new IllegalArgumentException("a time on the robot's clock, in seconds: " + seconds);
    }
    return Math.round(seconds * NANOS_PER_SECOND);
  }

  /** A time in nanoseconds, in seconds. */
  public static double toSeconds(long nanos) {
    return (double) nanos / NANOS_PER_SECOND;
  }
}
