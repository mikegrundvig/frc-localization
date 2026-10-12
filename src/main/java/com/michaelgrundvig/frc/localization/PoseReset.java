package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose2d;

/**
 * A pose the robot is told outright, with how far to trust it: placed on its start before the
 * match, or a driver's "the robot is against the wall here". The estimator weighs it like any
 * measurement, instead of jumping to it blindly.
 *
 * @param timestampNanos when the robot was there
 * @param pose where it was
 * @param covariance how sure the teller is
 * @param reason why, for the log: "placed on Center", "driver: against the wall"
 */
public record PoseReset(
    long timestampNanos, Pose2d pose, PoseCovariance covariance, String reason) {
  public PoseReset {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (reason.isBlank()) {
      throw new IllegalArgumentException("a pose reset needs its reason");
    }
  }
}
