package com.michaelgrundvig.frc.localization;

/**
 * What turned the robot's odometry over a step: the gyro, the backup IMU while the gyro is lost, or
 * the wheels when neither reports. Each IMU's scale is its own, and wrong its own way, so the
 * estimator keeps each one's error apart; the wheels' turning drifts far faster than either.
 */
public enum YawSource {
  GYRO,
  BACKUP,
  WHEELS
}
