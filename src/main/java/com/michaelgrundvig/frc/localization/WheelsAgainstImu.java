package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/**
 * Gives the {@link SlipCheck} each loop's wheel readings and what the IMU that turns odometry felt,
 * so it can say whether the wheels slid and whether the IMU felt a collision.
 *
 * <p>The slip check compares one loop with the next, so it needs the same IMU to have read both. A
 * loop without wheel readings, a loop without an IMU, and a loop read by another IMU than the last
 * each start it over.
 */
final class WheelsAgainstImu {
  private final SlipCheck slipCheck;

  /** Where each IMU is mounted, and whether its accelerometer reads along the robot's axes. */
  private final Imus imus;

  // The last wheel reading of the last loop that had any: when it was read, which IMU read its
  // yaw, and what that yaw was. The next loop's turn and time are measured from them.
  private long lastStepNanos = Long.MIN_VALUE;
  private YawSource lastStepYawFrom = YawSource.WHEELS;
  private double lastStepYaw = Double.NaN;

  WheelsAgainstImu(SlipCheck slipCheck, Imus imus) {
    this.slipCheck = slipCheck;
    this.imus = imus;
  }

  /** What the comparison found. */
  SlipCheck slipCheck() {
    return slipCheck;
  }

  /**
   * One loop.
   *
   * @param loop the loop's steps, and which IMU their yaws came from
   * @param imu what that IMU read this loop, or null when no IMU turns odometry
   * @param wheelVelocities the robot's velocity as its wheels measure it, robot-relative
   */
  void compare(
      OdometrySteps.Loop loop, @Nullable ImuReadings imu, ChassisVelocities wheelVelocities) {
    // What the IMU felt of the robot's own motion, along the robot's axes. Not a number without
    // an IMU.
    double feltForward = Double.NaN;
    double feltLeft = Double.NaN;
    if (imu != null) {
      feltForward = forwardAccelerationWithoutGravity(imu);
      feltLeft = leftAccelerationWithoutGravity(imu);
    }
    List<OdometrySteps.Step> steps = loop.steps();
    if (steps.isEmpty() || imu == null) {
      slipCheck.startOver(feltForward, feltLeft);
      return;
    }
    OdometrySteps.Step last = steps.get(steps.size() - 1);
    YawSource yawFrom = loop.yawFrom();
    boolean sameImuAsLastLoop = yawFrom == lastStepYawFrom;
    boolean timeHasPassed = lastStepNanos < last.timestampNanos();
    if (sameImuAsLastLoop && timeHasPassed) {
      double wheelTurn = 0;
      for (OdometrySteps.Step step : steps) {
        wheelTurn += step.motion().dthetaRadians();
      }
      ImuMount mount = mountOf(yawFrom);
      slipCheck.update(
          new SlipCheck.Sample(
              Timestamps.toSeconds(last.timestampNanos() - lastStepNanos),
              wheelTurn,
              MathUtil.angleModulus(last.yawRadians() - lastStepYaw),
              last.yawRadians(),
              imu.yawRateRadiansPerSecond(),
              mount.position(),
              mount.accelerometerChecked(),
              wheelVelocities,
              feltForward,
              feltLeft,
              tiltFromLevel(imu)));
    } else {
      slipCheck.startOver(feltForward, feltLeft);
    }
    lastStepNanos = last.timestampNanos();
    lastStepYawFrom = yawFrom;
    lastStepYaw = last.yawRadians();
  }

  /**
   * Where an IMU is mounted. Readings from a backup IMU the robot's definition does not have come
   * from an IMU nobody placed, which is taken to sit at the center with its accelerometer not
   * checked, as on a new robot.
   */
  private ImuMount mountOf(YawSource imu) {
    if (imu == YawSource.GYRO) {
      return imus.gyro();
    }
    return imus.backup().orElse(ImuMount.UNCHECKED);
  }

  /**
   * How far the robot is tilted from level, by an IMU's attitude: the angle between the robot's up
   * and straight up, whose cosine is the product of the pitch's and the roll's.
   */
  static double tiltFromLevel(ImuReadings imu) {
    return Math.acos(Math.min(1, Math.cos(imu.pitchRadians()) * Math.cos(imu.rollRadians())));
  }

  // What an IMU felt of the robot's own motion is its reading less gravity's share, which it feels
  // along the robot's axes as the robot tilts. The signs follow WPILib's attitude: a positive
  // pitch is nose down, a positive roll the left side up. In m/s²; not a number where the IMU
  // cannot say.

  private static double forwardAccelerationWithoutGravity(ImuReadings imu) {
    return imu.forwardAccelerationMetersPerSecSq()
        + ImuReadings.STANDARD_GRAVITY * Math.sin(imu.pitchRadians());
  }

  private static double leftAccelerationWithoutGravity(ImuReadings imu) {
    return imu.leftAccelerationMetersPerSecSq()
        - ImuReadings.STANDARD_GRAVITY * Math.sin(imu.rollRadians()) * Math.cos(imu.pitchRadians());
  }
}
