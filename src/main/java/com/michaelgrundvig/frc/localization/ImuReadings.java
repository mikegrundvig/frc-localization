package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * What one IMU read over one robot loop: whether it's reporting, its attitude and the jolt it felt
 * as of the loop, and each yaw reading taken since the last loop, with when it was taken (on a
 * robot, faster than the loop, so odometry turns with every one: {@link YawSamples}). Built by
 * name, so pitch and roll can't trade places: {@code
 * ImuReadings.builder().reporting(true).yawRadians(...)...build()}.
 *
 * @param reporting whether the IMU is connected and answering
 * @param yawRadians its yaw as of the loop, counterclockwise from wherever it started
 * @param yawRateRadiansPerSecond how fast it's turning as of the loop, counterclockwise: an IMU off
 *     the robot's center feels the turn carry it around
 * @param pitchRadians the robot's pitch, in WPILib's convention: turning about its left axis, so
 *     positive is nose down
 * @param rollRadians the robot's roll, turning about its forward axis
 * @param forwardAccelerationMetersPerSecSq acceleration along the robot's forward axis, as the IMU
 *     feels it: over the loop, as its filter reads it, with gravity's share as the robot tilts. Not
 *     a number from an IMU that can't say.
 * @param leftAccelerationMetersPerSecSq acceleration along the robot's left axis
 * @param yaws each yaw reading since the last loop, with when it was taken, oldest first
 */
@ConfigRecord
public record ImuReadings(
    boolean reporting,
    double yawRadians,
    double yawRateRadiansPerSecond,
    double pitchRadians,
    double rollRadians,
    double forwardAccelerationMetersPerSecSq,
    double leftAccelerationMetersPerSecSq,
    YawSamples yaws) {
  /**
   * Standard gravity, in m/s²: what an IMU at rest feels holding it up, and its share along the
   * robot's axes as the robot tilts.
   */
  public static final double STANDARD_GRAVITY = 9.80665;

  /** An IMU that isn't there. */
  public static final ImuReadings NONE =
      builder()
          .reporting(false)
          .yawRadians(0)
          .yawRateRadiansPerSecond(0)
          .pitchRadians(0)
          .rollRadians(0)
          .forwardAccelerationMetersPerSecSq(0)
          .leftAccelerationMetersPerSecSq(0)
          .yaws(YawSamples.NONE)
          .build();

  public static ImuReadingsBuilder.ReportingStage builder() {
    return ImuReadingsBuilder.builder();
  }

  /**
   * Whether it can be read from: reporting, and every value a number. An IMU that sends a value
   * that isn't one counts as lost, so a bad reading costs its readings, never the robot program.
   */
  public boolean usable() {
    if (!reporting
        || !Double.isFinite(yawRadians)
        || !Double.isFinite(pitchRadians)
        || !Double.isFinite(rollRadians)) {
      return false;
    }
    return yaws.areAllNumbers();
  }
}
