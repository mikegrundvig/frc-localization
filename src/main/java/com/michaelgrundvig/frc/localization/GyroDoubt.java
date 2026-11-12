package com.michaelgrundvig.frc.localization;

/**
 * How unsure the filter is of the gyro's drift, and how far both of the gyro's errors may wander,
 * where it estimates them.
 *
 * @param driftStdDevRadiansPerSecond how fast the gyro may drift steadily, before any camera has
 *     said anything of it
 * @param scaleWanderPerSqrtSecond how far the scale may have wandered after one second, as a
 *     standard deviation: its variance grows with time
 * @param driftWanderPerSqrtSecond the same for the drift, in radians a second
 */
record GyroDoubt(
    double driftStdDevRadiansPerSecond,
    double scaleWanderPerSqrtSecond,
    double driftWanderPerSqrtSecond) {
  /** A gyro whose drift is not estimated, and whose errors stay what they are. */
  static final GyroDoubt NONE = new GyroDoubt(0, 0, 0);
}
