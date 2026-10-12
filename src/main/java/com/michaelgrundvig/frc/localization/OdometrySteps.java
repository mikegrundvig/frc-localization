package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * Turns each loop's wheel readings into the steps the {@link PoseEstimator} takes: how the robot
 * moved between one reading and the next, and its yaw at each reading.
 *
 * <p>The yaw comes from the gyro while it has readings, from the backup IMU while the gyro is lost,
 * and from neither when both are lost. An IMU read less often than the wheels still gives a yaw for
 * every wheel reading, interpolated between its own readings ({@link YawTrack}). With no IMU the
 * steps carry no yaw, and the estimator turns them by the wheels' own turning, which it knows
 * drifts.
 *
 * <p>The estimator is told which IMU the yaws came from, so it keeps each IMU's scale error apart,
 * and so it turns the step where the source changes by the wheels rather than take the two IMUs'
 * different zeros as a turn.
 */
final class OdometrySteps {
  /**
   * One wheel reading, as the estimator takes it.
   *
   * @param timestampNanos when the wheels were read
   * @param motion how the robot moved since the reading before
   * @param yawRadians the yaw then, from the loop's IMU; not a number when no IMU read it
   */
  record Step(long timestampNanos, BodyMotion motion, double yawRadians) {}

  /**
   * One loop's steps.
   *
   * @param steps a step for each wheel reading, oldest first
   * @param yawFrom which IMU their yaws came from, or the wheels when neither IMU had readings
   */
  record Loop(List<Step> steps, YawSource yawFrom) {}

  private final DriveKinematicsModel kinematics;
  private final YawTrack gyroYaws = new YawTrack();
  private final YawTrack backupYaws = new YawTrack();

  /** The previous wheel reading, which the next one's motion is measured from. */
  private @Nullable OdometrySample lastReading;

  /** What the yaws came from in the last loop that had wheel readings. */
  private YawSource yawFromInLastLoopWithWheels = YawSource.GYRO;

  OdometrySteps(DriveKinematicsModel kinematics) {
    this.kinematics = kinematics;
  }

  /**
   * A loop's steps, from every wheel reading since the last loop and what both IMUs read. The very
   * first wheel reading has nothing to measure a motion from, so its step has none.
   */
  Loop take(List<OdometrySample> wheels, ImuReadings gyro, ImuReadings backup) {
    keepThisLoopsYaws(gyroYaws, gyro);
    keepThisLoopsYaws(backupYaws, backup);
    YawSource yawFrom = whichImuHasReadings();
    if (!wheels.isEmpty()) {
      yawFromInLastLoopWithWheels = yawFrom;
    }
    List<Step> steps = new ArrayList<>(wheels.size());
    for (OdometrySample reading : wheels) {
      OdometrySample previous = lastReading;
      BodyMotion motion =
          previous == null ? new BodyMotion(0, 0, 0, 0) : kinematics.motion(previous, reading);
      steps.add(new Step(reading.timestampNanos(), motion, yawAt(yawFrom, reading)));
      lastReading = reading;
    }
    return new Loop(steps, yawFrom);
  }

  /** What turns odometry now: what the yaws came from in the last loop that had wheel readings. */
  YawSource turnsOdometryNow() {
    return yawFromInLastLoopWithWheels;
  }

  /** The gyro if it has readings this loop, else the backup IMU if it has, else neither. */
  private YawSource whichImuHasReadings() {
    if (gyroYaws.hasReadings()) {
      return YawSource.GYRO;
    }
    if (backupYaws.hasReadings()) {
      return YawSource.BACKUP;
    }
    return YawSource.WHEELS;
  }

  private double yawAt(YawSource yawFrom, OdometrySample reading) {
    return switch (yawFrom) {
      case GYRO -> gyroYaws.at(reading.timestampNanos());
      case BACKUP -> backupYaws.at(reading.timestampNanos());
      case WHEELS -> Double.NaN;
    };
  }

  /** Takes an IMU's yaw readings this loop, or forgets them all while it is lost. */
  private static void keepThisLoopsYaws(YawTrack yaws, ImuReadings imu) {
    YawSamples readings = imu.yaws();
    int count = readings.count();
    if (!imu.usable() || count == 0) {
      yaws.clear();
      return;
    }
    long[] times = new long[count];
    double[] values = new double[count];
    for (int i = 0; i < count; i++) {
      times[i] = readings.timeNanos(i);
      values[i] = readings.yawRadians(i);
    }
    yaws.update(times, values);
  }
}
