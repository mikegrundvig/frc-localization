package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where the robot is, as of one loop: the pose, how sure the estimator is, how the robot is moving,
 * and how healthy localization is. Gates, dashboards, and drivers act on the mode and health, not
 * only the pose: a precise-looking pose from dead reckoning is not one to aim with.
 *
 * @param timestampNanos the time of the newest odometry sample it includes
 * @param pose the robot's pose on the field
 * @param covariance its uncertainty
 * @param fieldVelocity meters per second, in the field frame
 * @param yawRateRadiansPerSecond how fast the robot is turning, counterclockwise
 * @param mode what the estimate is currently based on
 * @param health the numbers behind the mode
 */
public record LocalizationEstimate(
    long timestampNanos,
    Pose2d pose,
    PoseCovariance covariance,
    Translation2d fieldVelocity,
    double yawRateRadiansPerSecond,
    Mode mode,
    Health health) {
  /** What the estimate is currently based on. */
  public enum Mode {
    /** Not yet placed or seen: the pose is a placeholder. */
    INITIALIZING,
    /** Recent camera measurements agree with odometry. */
    TRACKING,
    /** No recent camera measurement: odometry alone, with uncertainty growing. */
    DEAD_RECKONING,
    /** Camera measurements agree with each other but not with the estimate; recovering. */
    LOST,
    /** The gyro is lost: heading comes from the wheels, and drifts fast. */
    IMU_FAULT
  }

  /**
   * The numbers behind the mode.
   *
   * @param visionAgeSeconds since the newest image the estimate rests on was taken: the newest
   *     accepted, or that agreed on a recovery; infinite if none yet
   * @param accepted camera measurements accepted since the robot started
   * @param rejected camera measurements rejected since the robot started
   * @param lastInnovationMeters how far the last compared measurement was from the estimate
   * @param recentDisagreementMeters the farthest any camera measurement accepted lately (within
   *     {@link EstimatorConfig#disagreementWindowSeconds}) was from the estimate it corrected: how
   *     well the cameras and odometry agree. NaN when none was accepted lately.
   * @param residualMeters the wheels' disagreement in the newest odometry step
   * @param suspectTravelMeters how much of the robot's recent travel odometry doubts: slip, tilt,
   *     and the wheels' disagreement, adding up while it lasts
   * @param tiltRadians how far the robot was tilted from level at the newest step
   * @param collisionActive whether a collision's window of doubt covers the newest step
   * @param recoveries how many times the estimate has jumped to agreeing measurements it had
   *     rejected
   * @param gyroConnected whether an IMU turned the newest step: the gyro, or the backup IMU while
   *     the gyro is lost. False only when neither did, and the wheels turned it.
   */
  public record Health(
      double visionAgeSeconds,
      int accepted,
      int rejected,
      double lastInnovationMeters,
      double recentDisagreementMeters,
      double residualMeters,
      double suspectTravelMeters,
      double tiltRadians,
      boolean collisionActive,
      int recoveries,
      boolean gyroConnected) {}

  /**
   * Where the robot will be after some time at its current velocity and turn rate: for acting
   * before arriving (predictive zones, aiming from the pose a shot will leave from).
   */
  public Pose2d predict(double seconds) {
    Translation2d moved = fieldVelocity.times(seconds);
    return new Pose2d(
        pose.getTranslation().plus(moved),
        pose.getRotation().plus(new Rotation2d(yawRateRadiansPerSecond * seconds)));
  }
}
