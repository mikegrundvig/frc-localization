package com.michaelgrundvig.frc.localization;

/**
 * What the estimate's mode is decided by.
 *
 * @param gyroConnected whether an IMU turned the newest step
 * @param placed whether the robot has been told where it is, or found by a camera
 * @param camerasKeepDisagreeing whether the cameras have kept disagreeing, measurement after
 *     measurement
 * @param visionRecent whether a camera measurement was accepted recently
 */
record ModeFacts(
    boolean gyroConnected, boolean placed, boolean camerasKeepDisagreeing, boolean visionRecent) {
  /**
   * What the estimate rests on, in order: a lost gyro, then not placed yet, then cameras that keep
   * disagreeing, then recent camera measurements, else odometry alone.
   */
  LocalizationEstimate.Mode mode() {
    if (!gyroConnected) {
      return LocalizationEstimate.Mode.IMU_FAULT;
    }
    if (!placed) {
      return LocalizationEstimate.Mode.INITIALIZING;
    }
    if (camerasKeepDisagreeing) {
      return LocalizationEstimate.Mode.LOST;
    }
    if (visionRecent) {
      return LocalizationEstimate.Mode.TRACKING;
    }
    return LocalizationEstimate.Mode.DEAD_RECKONING;
  }
}
