package com.michaelgrundvig.frc.localization;

import io.soabase.recordbuilder.core.RecordBuilder;

/**
 * How far localization trusts the wheels this loop, and why: what the IMU that turns odometry says
 * of them, through the {@link SlipCheck}. The slip check judges the wheels together, as they slide
 * together over the floor (a wheel slipping against the others shows in the kinematics' residual
 * instead, {@link BodyMotion#residualMeters}), so this is one record for the whole drivetrain.
 *
 * @param imu the IMU that said all this, the one that turns odometry this loop: the gyro, the
 *     backup IMU while the gyro is lost, or none ({@link YawSource#WHEELS})
 * @param conditions what the estimator weighs each wheel step by: the robot's tilt, the ground's
 *     slope under it, a jolt's share of a collision, how fast the wheels may be sliding, and
 *     whether the robot is disabled
 * @param turnSlipMetersPerSecond how fast the wheels may be sliding, as the gyro sees them turning
 * @param speedSlipMetersPerSecond how fast they may be sliding, as the accelerometer sees their
 *     speed
 * @param turnApartRadians how far the wheels turned the robot beyond the gyro over the slip check's
 *     window, either way
 * @param slideMetersPerSecond how much faster or slower than the robot the wheels roll, as the
 *     accelerometer reckons it
 * @param unexplainedMetersPerSecSq what the IMU felt that the wheels' own change of velocity
 *     doesn't explain
 * @param accelerometerBiasMetersPerSecSq the accelerometer's bias, learned while the robot stands
 *     still
 * @param accelerometerLeaksGravity whether the accelerometer reads gravity's share standing still,
 *     past any bias a part has: the IMU's orientation in the robot's definition isn't how it's
 *     mounted
 */
@OutputRecord
public record WheelTrust(
    @RecordBuilder.Initializer(value = "IMU", source = WheelTrust.class) YawSource imu,
    @RecordBuilder.Initializer(value = "NORMAL", source = DriveConditions.class)
        DriveConditions conditions,
    double turnSlipMetersPerSecond,
    double speedSlipMetersPerSecond,
    double turnApartRadians,
    double slideMetersPerSecond,
    double unexplainedMetersPerSecSq,
    double accelerometerBiasMetersPerSecSq,
    boolean accelerometerLeaksGravity) {
  /** No IMU said anything: what a builder leaves the IMU as. */
  public static final YawSource IMU = YawSource.WHEELS;

  /** Builds one by name: no IMU, normal conditions, and nothing sliding, unless said. */
  public static WheelTrustBuilder builder() {
    return WheelTrustBuilder.builder();
  }
}
