package com.michaelgrundvig.frc.localization;

import org.wpilib.math.util.MathUtil;

/**
 * The IMUs' readings, unwrapped into one continuous yaw that never wraps.
 *
 * <p>An IMU turns a step only when it also read the step before. A step after the IMU was lost,
 * came back (its reading may have jumped), or took over from the other IMU (which has a zero of its
 * own) is turned by the wheels, and the IMU's next steps carry on from its new reading.
 */
final class ContinuousYaw {
  /** The continuous yaw as of the newest step, in radians. */
  private double radians;

  /** The last reading an IMU gave, as it gave it: the next reading's turn is measured from it. */
  private double lastImuReading;

  /** What read the yaw at the newest step: an IMU, or the wheels when none did. */
  private YawSource lastReader = YawSource.WHEELS;

  /** What read the yaw: the IMU named, or the wheels if none was or its reading isn't a number. */
  static YawSource readerOf(YawSource source, double yawRadians) {
    if (source == YawSource.WHEELS || !Double.isFinite(yawRadians)) {
      return YawSource.WHEELS;
    }
    return source;
  }

  /** Starts the yaw at the first step: the IMU's reading, or zero if the wheels read it. */
  void start(YawSource readBy, double yawRadians) {
    radians = readBy == YawSource.WHEELS ? 0 : yawRadians;
    lastImuReading = radians;
    lastReader = readBy;
  }

  /**
   * What turns a step: the IMU that read its yaw, if the same IMU read the step before too, so the
   * difference between the two readings is a turn. Otherwise the wheels' own turning stands in.
   */
  YawSource turnerOf(YawSource readBy) {
    boolean sameImuReadTheStepBefore = readBy != YawSource.WHEELS && readBy == lastReader;
    if (sameImuReadTheStepBefore) {
      return readBy;
    }
    return YawSource.WHEELS;
  }

  /** Turns the yaw by the wheels' own turn over a step. */
  void turnByWheels(double turnRadians) {
    radians += turnRadians;
  }

  /**
   * Turns the yaw by what the IMU turned since its last reading, the short way around.
   *
   * @return the IMU's turn, in radians
   */
  double turnByImu(double yawRadians) {
    double imuTurn = MathUtil.angleModulus(yawRadians - lastImuReading);
    radians += imuTurn;
    return imuTurn;
  }

  /** Notes what read the yaw at the step just added, and the IMU's reading if one did. */
  void read(YawSource readBy, double yawRadians) {
    if (readBy != YawSource.WHEELS) {
      lastImuReading = yawRadians;
    }
    lastReader = readBy;
  }

  /** The continuous yaw as of the newest step, in radians. */
  double radians() {
    return radians;
  }

  /** What read the yaw at the newest step. */
  YawSource lastReader() {
    return lastReader;
  }
}
