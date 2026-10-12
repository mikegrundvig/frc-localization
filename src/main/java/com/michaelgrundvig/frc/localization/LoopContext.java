package com.michaelgrundvig.frc.localization;

/**
 * One robot loop, as {@link LocalizationService} is told it with the drivetrain's readings: when
 * the loop ran, on the robot's clock, and whether the robot was enabled. The service reads no
 * clock; this is its time.
 *
 * @param nowNanos the robot's clock this loop, in nanoseconds ({@code RobotController.getTime()})
 * @param enabled whether the robot is enabled; while disabled, people may carry it where its wheels
 *     can't see
 */
public record LoopContext(long nowNanos, boolean enabled) {
  public LoopContext {
    if (nowNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + nowNanos);
    }
  }
}
